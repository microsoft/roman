from math import inf
from multiprocessing import Value
import numpy as np
import time
from .rq.hand import GraspMode, Hand, Position
from . import rq
from .ur.arm import Arm, JointSpeeds, Tool, Joints
from . import ur
from . import server
from .sim.simscene import SimScene
from .ur import UR_DEFAULT_FORCE_LOW_BOUND, UR_DEFAULT_FORCE_HI_BOUND, UR_DEFAULT_MAX_SPEED, UR_DEFAULT_ACCELERATION

FORCE_LIMIT_DEFAULT = (UR_DEFAULT_FORCE_LOW_BOUND, UR_DEFAULT_FORCE_HI_BOUND)
FORCE_LIMIT_TOUCH = ([-5, -5, -5, -0.5, -0.5, -0.5], [5, 5, 5, 0.5, 0.5, 0.5])
NO_FORCE_LIMIT = (None, None)

def _default_completion_condition(arm_state, hand_state):
    return arm_state.is_done() and hand_state.is_done()

class Robot:
    '''
    Combines the manipulator components (arm, hand, FT and tactile sensors).
    '''

    def __init__(self, use_sim=True, writer=None, config={}):
        self.use_sim = use_sim
        self.config = config
        self._writer = writer

    def connect(self):
        '''
        Connects the robot instance to either sim or real (hardware) backing.
        '''
        self._server = server.create(self.use_sim, self.config)
        self._server.connect()
        self.active_force_limit = self.config.get("force_limit", FORCE_LIMIT_DEFAULT)
        self.arm = Arm(self._server.arm)
        self.hand = Hand(self._server.hand)
        self.arm.stop()
        self.hand.read()
        return self

    def disconnect(self):
        self._server.disconnect()

    @property
    def tool_pose(self):
        return self.arm.state.tool_pose().clone()

    @property
    def tool_speed(self):
        return self.arm.state.tool_speed().clone()

    @property
    def joint_positions(self):
        return self.arm.state.joint_positions().clone()

    @property
    def joint_speeds(self):
        return self.arm.state.joint_speeds()().clone()

    @property
    def force(self):
        return self.arm.state.sensor_force().clone()

    @property
    def has_object(self):
        return self.hand.state.object_detected()

    def move_simple(self, dx, dy, dz, dyaw, max_speed=0.5, max_acc=0.5, timeout=0.2):
        '''
        Moves the arm relative to the current position in carthesian coordinates,
        assuming the gripper is vertical (aligned with the z-axis), pointing down.
        This version returns after the amount of time specified by dt.
        '''
        self.arm.read()
        pose = self.arm.state.tool_pose()
        pose = Tool.from_xyzrpy(pose.to_xyzrpy() + [dx, dy, dz, 0, 0, dyaw])
        self.arm.move(pose, max_speed=max_speed, max_acc=max_acc, blocking=False)
        self.__complete_move(timeout, None)

    def convert(self, target):
        self.arm.move(target, max_speed=0, blocking=False)
        self.arm.read()
        if type(target) is Joints:
            return self.arm.state.target_tool_pose()
        return self.arm.state.target_joint_positions()

    def move(self,
             target,
             max_speed=UR_DEFAULT_MAX_SPEED,
             max_acc=UR_DEFAULT_ACCELERATION,
             force_limit=None,
             timeout=None,
             completion=None):
        force_limit = force_limit or self.active_force_limit
        self.arm.move(target,
                      max_speed=max_speed,
                      max_acc=max_acc,
                      force_low_bound=force_limit[0],
                      force_high_bound=force_limit[1],
                      blocking=False)
        return self.__complete_move(timeout, completion)

    def touch(self,
              target,
              max_speed=UR_DEFAULT_MAX_SPEED,
              max_acc=UR_DEFAULT_ACCELERATION,
              force_limit=FORCE_LIMIT_TOUCH,
              timeout=None,
              completion=None):
        force_limit = force_limit or self.active_force_limit
        self.arm.touch(target,
                       max_speed=max_speed,
                       max_acc=max_acc,
                       force_low_bound=force_limit[0],
                       force_high_bound=force_limit[1],
                       blocking=False)
        return self.__complete_move(timeout, completion)

    def set_hand_mode(self, mode=GraspMode.BASIC):
        self.hand.set_mode(mode)

    def grasp(self, position=Position.CLOSED, speed=255, force=0, timeout=None, completion=None):
        self.hand.set_mode(GraspMode.BASIC)
        self.hand.move(position=position, speed=speed, force=force, blocking=False)
        return self.__complete_move(timeout, completion)

    def pinch(self, position=Position.CLOSED, speed=255, force=0, timeout=None, completion=None):
        self.hand.set_mode(GraspMode.PINCH)
        self.hand.move(position=position, speed=speed, force=force, blocking=False)
        return self.__complete_move(timeout, completion)

    def open(self, position=Position.OPENED, speed=255, force=0, timeout=None, completion=None):
        self.hand.move(position=position, speed=speed, force=force, blocking=False)
        return self.__complete_move(timeout, completion)

    def release(self, position=Position.OPENED, speed=0, force=255, timeout=None, completion=None):
        self.hand.move(position=position, speed=speed, force=force, blocking=False)
        return self.__complete_move(timeout, completion)

    def move_finger(self, position, finger=rq.hand.Finger.All, speed=255, force=0, timeout=None, completion=None):
        timeout = timeout if timeout is not None else np.inf
        self.hand.move(position=position, finger=finger, speed=speed, force=force, blocking=False)
        return self.__complete_move(timeout, completion)

    def stop(self, timeout=None):
        self.arm.stop(blocking=False)
        self.hand.stop(blocking=False)
        return self.__complete_move(timeout, None)

    def execute(self, arm_cmd=ur.arm.Command(), hand_cmd=rq.hand.Command(), timeout=0, completion=None):
        self.arm.execute(arm_cmd, blocking=False)
        self.hand.execute(hand_cmd, blocking=False)
        return self.__complete_move(timeout, completion)

    def read(self):
        self.arm.read()
        self.hand.read()
        self.__log()
        return self.arm.state.clone(), self.hand.state.clone()

    def last_command(self):
        return self.arm.command.clone(), self.hand.command.clone()

    def last_state(self):
        return self.arm.state.clone(), self.hand.state.clone()

    def is_done(self):
        return self.arm.state.is_done() and self.hand.state.is_done()

    def step(self):
        self.arm.step()
        self.hand.step()
        self.__log()

    def wait(self, timeout):
        self.arm.read()
        end = self.arm.state.time() + timeout
        while self.arm.state.time() < end:
            self.read()

    def __complete_move(self, timeout, completion):
        completion = completion or _default_completion_condition
        self.__log()
        endtime = self.arm.state.time() + timeout if timeout is not None else inf
        while not completion(self.arm.state, self.hand.state) and self.arm.state.time() < endtime:
            self.step()
        return completion(self.arm.state, self.hand.state)

    def __log(self):
        if self._writer is not None:
            self._writer(self.arm.state, self.hand.state, self.arm.command, self.hand.command)


def connect(config={}):
    '''
    Creates and returns a robot instance with real (hardware) backing.
    '''
    return Robot(use_sim=False, config=config).connect()


def connect_sim(scene_init_fn=None, config={}):
    '''
    Creates and returns a simulated robot instance together with a sim scene manager.
    '''
    r = Robot(use_sim=True, config=config).connect()
    s = SimScene(r, scene_init_fn).connect()
    return r, s
