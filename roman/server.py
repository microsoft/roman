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
        self.__process = Process(target=server,
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

def client_loop(client, shared_cmd, shared_state, lock, shutdown_event):
    global last_client_pulse
    local_cmd = shared_cmd.clone()
    local_state = shared_state.clone()
    while not shutdown_event.is_set():
        if client.poll(MIN_CLIENT_REQ_INTERVAL):
            last_client_pulse = time.perf_counter()
            client.recv_bytes_into(local_cmd.array) 
            
            with lock:
                shared_cmd[:] = local_cmd
                local_state[:] = shared_state
                
            client.send_bytes(local_state.array)

def control_loop(controller, shared_cmd, shared_state, lock, shutdown_event, keep_alive_cmd=None):
    '''
    Control loop running as fast as the hardware controller allows (best effort but not guaranteed).
    For e-series UR arms, 250Hz (4ms loop). For UR CB2, 125Hz (8ms loop). For Robotiq hands, 100Hz (10ms loop).
    This enables high(er)-speed closed-loop control using force and tactile sensing (but no vision).
    '''
    global last_client_pulse
    local_cmd = shared_cmd.clone()
    local_state = shared_state.clone()
    
    while not shutdown_event.is_set():
        with lock:
            shared_state[:] = local_state
            local_cmd[:] = shared_cmd
            
        # only move the hardware as long as the client is sending us commands  
        if time.perf_counter() - last_client_pulse  < MIN_CLIENT_REQ_INTERVAL:
            controller.execute(local_cmd, local_state)
        elif keep_alive_cmd is not None:
            controller.execute(keep_alive_cmd, local_state)

def server(arm_client, hand_client, shutdown_event, reset_event, robot_type, config):
    robot = robot_type(config)
    robot.connect()

    lock = Lock()
    cmd = ur.Command()
    state = ur.State()
    arm_control_thread = Thread(target=control_loop, args=(robot.arm, cmd, state, lock, shutdown_event, ur.Command().make_estop()))
    arm_control_thread.start()
    arm_client_thread = Thread(target=client_loop, args=(arm_client, cmd, state, lock, shutdown_event))
    arm_client_thread.start()

    lock = Lock()
    cmd = rq.Command()
    state = rq.State()
    hand_control_thread = Thread(target=control_loop, args=(robot.hand, cmd, state, lock, shutdown_event))
    hand_control_thread.start()
    hand_client_thread = Thread(target=client_loop, args=(hand_client, cmd, state, lock, shutdown_event))
    hand_client_thread.start()

    shutdown_event.wait()
        
    arm_control_thread.join()
    arm_client_thread.join()
    hand_control_thread.join()
    hand_client_thread.join()

    robot.disconnect()