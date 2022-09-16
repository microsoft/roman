from threading import Thread
import time
from multiprocessing import Lock, Process, Pipe, Event

from . import rq
from . import ur
from .sim.simenv import SimEnv

def create(use_sim, config):
    '''
    Creates the appropriate server for either sim or real (hardware) backing.
    '''
    server_type = SimRobotServer if use_sim else RealRobotServer
    return RemoteRobotProxy(server_type, config)


class InProcRobotServer():
    def __init__(self, config):
        self.activate_hand = config.get("hand.activate", True)

    def connect(self):
        if self.activate_hand:
            self._hand_con.connect()
            self.hand = rq.HandController(self._hand_con)
        else:
            self.hand = rq.NoHandController(self._hand_con)
        self._arm_con.connect()
        self.arm = ur.ArmController(self._arm_con)

    def disconnect(self):
        self._arm_con.disconnect()
        if self.activate_hand:
            self._hand_con.disconnect()

    def update(self):
        pass


class RealRobotServer(InProcRobotServer):
    def __init__(self, config):
        super().__init__(config)
        self._arm_con = ur.Connection()
        self._hand_con = rq.Connection()


class SimRobotServer(InProcRobotServer):
    def __init__(self, config):
        super().__init__(config)
        self.env = SimEnv(config)
        self._arm_con = ur.SimConnection(self.env)
        self._hand_con = rq.SimConnection(self.env)

    def connect(self):
        self.env.connect()
        super().connect()

    def disconnect(self):
        super().disconnect()
        self.env.disconnect()

    def reset(self):
        self.env.reset()

    def update(self):
        self.env.update()


class RemoteRobotProxy():
    class PipeConnection:
        def __init__(self, pipe):
            self.pipe = pipe

        def execute(self, cmd, state):
            self.pipe.send_bytes(cmd.array)
            self.pipe.recv_bytes_into(state.array)

    def __init__(self, robot_type, config):
        self.__robot_type = robot_type
        self.__config = config

    def connect(self):
        hand_server, hand_client = Pipe(duplex=True)
        arm_server, arm_client = Pipe(duplex=True)
        self.__shutdown_event = Event()
        self.__reset_event = Event()
        self.__reset_event.set()
        self.__process = Process(target=server_loop,
                                 args=(arm_client,
                                       hand_client,
                                       self.__shutdown_event,
                                       self.__reset_event,
                                       self.__robot_type,
                                       self.__config),
                                 name="roman_controller",
                                 daemon=True)
        self.__process.start()
        self.arm = RemoteRobotProxy.PipeConnection(arm_server)
        self.hand = RemoteRobotProxy.PipeConnection(hand_server)
        return (self.arm, self.hand)

    def disconnect(self):
        self.__shutdown_event.set()
        self.__process.join()

    def reset(self):
        self.__reset_event.clear()
        self.__reset_event.wait()


#************************************************************************************************
# Server loop
#************************************************************************************************
last_client_pulse = 0
MIN_CLIENT_REQ_INTERVAL = 0.5 #seconds
def client_loop(client, cmd, state, lock, shutdown_event):
    global last_client_pulse
    local_cmd = cmd.clone()
    local_state = state.clone()
    while not shutdown_event.is_set():
        if client.poll(1):
            last_client_pulse = time.perf_counter()
            client.recv_bytes_into(local_cmd.array) 
            with lock:
                cmd[:] = local_cmd
                local_state[:] = state
                
            client.send_bytes(local_state.array)

def server_loop(arm_client, hand_client, shutdown_event, reset_event, robot_type, config):
    '''
    Control loop running at 1/2 the frequency of the hardware (e.g. 250Hz on e-series) (best effort but not guaranteed).
    It enables high(er)-speed closed-loop control using force and tactile sensing (but no vision).
    '''
    global last_client_pulse
    robot = robot_type(config)
    robot.connect()

    shared_arm_cmd = ur.Command()
    shared_arm_state = ur.State()
    shared_hand_cmd = rq.Command()
    shared_hand_state = rq.State()
    local_arm_cmd = ur.Command()
    local_arm_state = ur.State()
    local_hand_cmd = rq.Command()
    local_hand_state = rq.State()
    lock = Lock()
    arm_client_thread = Thread(target=client_loop, args=(arm_client, shared_arm_cmd, shared_arm_state, lock, shutdown_event))
    arm_client_thread.start()
    hand_client_thread = Thread(target=client_loop, args=(hand_client, shared_hand_cmd, shared_hand_state, lock, shutdown_event))
    hand_client_thread.start()
    while not shutdown_event.is_set():
        if not reset_event.is_set():
            # primarily in support of sim, this enables restoring the environment to an initial state
            robot.reset()
            reset_event.set()
        
        with lock:
            shared_arm_state[:] = local_arm_state
            shared_hand_state[:] = local_hand_state
            local_arm_cmd[:] = shared_arm_cmd
            local_hand_cmd[:] = shared_hand_cmd
            
        if time.perf_counter() - last_client_pulse  > MIN_CLIENT_REQ_INTERVAL:
            # the client went dark, so stop moving the arm
            local_arm_cmd = ur.Command(local_arm_cmd.id())
            local_hand_cmd = rq.Command()
        robot.arm.execute(local_arm_cmd, local_arm_state)
        robot.hand.execute(local_hand_cmd, local_hand_state)


    arm_client_thread.join()
    hand_client_thread.join()

    # Disconnect the arm and gripper.
    robot.disconnect()
