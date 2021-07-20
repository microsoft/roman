import os
import numpy
import threading
import time
from multiprocessing import Process, Pipe, Event
from . import rq
from . import ur
from .sim.ur_rq3 import SimEnv

def start(config):
    con = InProcHost(config) if config.get("in_proc", False) else RemoteHostProxy(config)
    con.start()
    return con

class InProcHost:
    def __init__(self, config):
        self.config = config
        self.use_sim = config.get("use_sim", True)

    def start(self):
        if self.use_sim:
            self.env = self.config.get("sim.env", SimEnv)()
            self.env.connect()
            self._arm_con = ur.SimConnection(self.env)
            self._hand_con = rq.SimConnection(self.env)
            configurator = self.config.get("sim.init", None)
            if configurator is not None:
                configurator(self.env)

        else:
            self._arm_con = ur.Connection()
            self._hand_con = rq.Connection()

        self._arm_con.connect()
        activate_hand = self.config.get("hand.activate", True)
        self._hand_con.connect(activate_hand)
        self.arm = ur.ArmController(self._arm_con)
        self.hand = rq.HandController(self._hand_con)
        return (self.arm, self.hand)

    def stop(self):
        self._arm_con.disconnect()
        self._hand_con.disconnect()
        if self.use_sim:
            self.env.disconnect()

class RemoteHostProxy:
    class PipeConnection:
        def __init__(self, pipe):
            self.pipe = pipe
        def execute(self, cmd, state):
            self.pipe.send_bytes(cmd.array)
            self.pipe.recv_bytes_into(state.array)

    def __init__(self, config):
        self.config = config

    def start(self):
        hand_server, hand_client = Pipe(duplex=True)
        arm_server, arm_client = Pipe(duplex=True)
        self.__shutdown_event = Event()
        self.__process = Process(target=server_loop, args=(arm_client, hand_client, self.__shutdown_event, self.config))
        self.__process.start()
        self.arm = RemoteHostProxy.PipeConnection(arm_server)
        self.hand = RemoteHostProxy.PipeConnection(hand_server)
        return (self.arm, self.hand)

    def stop(self):
        self.__shutdown_event.set()
        self.__process.join()


#************************************************************************************************
# Server loop
#************************************************************************************************
def server_loop(arm_client, hand_client, shutdown_event, config={}, log_file=None, freq = 1./125):
    '''
    Control loop running at the same frequency as the hardware (e.g. 125Hz) (best effort but not guaranteed).
    It enables high-speed closed-loop control using force and tactile sensing (but no vision).
    '''
    if log_file is not None:
        file = open(log_file, "wb")
        #file.write(ur.UR_PROTOCOL_VERSION)

    host = InProcHost(config)
    (arm_ctrl, hand_ctrl) = host.start()

    arm_cmd = ur.Command()
    arm_state = ur.State()
    hand_cmd = rq.Command()
    hand_state = rq.State()
    while not shutdown_event.is_set():
        start_time = time.time()
        arm_cmd_is_new = arm_client.poll()
        if arm_cmd_is_new:
            arm_client.recv_bytes_into(arm_cmd.array) # blocking

        hand_cmd_is_new = hand_client.poll()
        if hand_cmd_is_new:
            hand_client.recv_bytes_into(hand_cmd.array) # blocking

        hand_ctrl.execute(hand_cmd, hand_state)
        arm_ctrl.execute(arm_cmd, arm_state)

        if arm_cmd_is_new:
            arm_client.send_bytes(arm_state.array)

        if hand_cmd_is_new:
            hand_client.send_bytes(hand_state.array)

        #log(file, arm_cmd, hand_cmd, arm_state, hand_state)
        if log_file is not None:
            file.write(arm_cmd, hand_cmd, arm_state, hand_state)

        if time.time()-start_time > 2*freq:
            print("Server loop lagging: " + str(time.time()-start_time))

    # Disconnect the arm and gripper.
    host.stop()

