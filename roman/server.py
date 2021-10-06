#import time
from multiprocessing import Process, Pipe, Event
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
        self._arm_con.connect()
        self._hand_con.connect(self.activate_hand)
        self.arm = ur.ArmController(self._arm_con)
        self.hand = rq.HandController(self._hand_con)

    def disconnect(self):
        self._arm_con.disconnect()
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
                                       self.__config))
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
def server_loop(arm_client, hand_client, shutdown_event, reset_event, robot_type, config):
    '''
    Control loop running at the same frequency as the hardware (e.g. 125Hz) (best effort but not guaranteed).
    It enables high(er)-speed closed-loop control using force and tactile sensing (but no vision).
    '''
    robot = robot_type(config)
    robot.connect()

    arm_cmd = ur.Command()
    arm_state = ur.State()
    hand_cmd = rq.Command()
    hand_state = rq.State()
    while not shutdown_event.is_set():
        #start_time = time.time()

        if not reset_event.is_set():
            # primarily in support of sim, this enables restoring the environment to an initial state
            robot.reset()
            reset_event.set()

        arm_cmd_is_new = arm_client.poll()
        if arm_cmd_is_new:
            arm_client.recv_bytes_into(arm_cmd.array) # blocking

        hand_cmd_is_new = hand_client.poll()
        if hand_cmd_is_new:
            hand_client.recv_bytes_into(hand_cmd.array) # blocking

        robot.hand.execute(hand_cmd, hand_state)
        robot.arm.execute(arm_cmd, arm_state)
        # robot.update()  # the hand (rq.sim_connection) already calls update()

        if arm_cmd_is_new:
            arm_client.send_bytes(arm_state.array)

        if hand_cmd_is_new:
            hand_client.send_bytes(hand_state.array)

        # if time.time()-start_time > 2*freq:
        #     print("Server loop lagging: " + str(time.time()-start_time))

    # Disconnect the arm and gripper.
    robot.disconnect()

