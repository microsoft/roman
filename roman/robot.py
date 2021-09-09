import os
import numpy as np
import threading
import time
from .rq.hand import Hand, Position
from . import rq
from .ur.arm import Arm, Tool, Joints
from . import ur
from . import server
from .sim.simscene import SimScene

class Robot:
    '''
    Combines the manipulator components (arm, hand, FT and tactile sensors).
    '''
    def connect(self, use_sim=True, config={}):
        '''
        Connects the robot instance to either sim or real (hardware) backing.
        '''
        self._use_sim = use_sim
        self._config = config
        self._server = server.create(use_sim, config)
        self._server.connect()
        self.arm = Arm(self._server.arm)
        self.hand = Hand(self._server.hand)
        self.arm.read()
        self.hand.read()
        return self

    def disconnect(self):
        self._server.disconnect()

    def move_simple(self, dx, dy, dz, dyaw, gripper_state=Position.OPENED, max_speed=0.5):
        '''
        Moves the arm relative to the current position in carthesian coordinates,
        assuming the gripper is vertical (aligned with the z-axis), pointing down.
        This supports the simplest Gym robotic manipulation environment.
        '''
        self.arm.read()
        pose = self.arm.state.tool_pose()
        pose = Tool.from_xyzrpy(pose.to_xyzrpy() + [dx, dy, dz, 0, 0, dyaw])
        self.arm.move(pose, max_speed=max_speed)
        self.hand.move(position=gripper_state)

    def step(self, dx, dy, dz, dyaw, gripper_state=Position.OPENED, max_speed=0.5, max_acc=0.5, dt=0.2):
        '''
        Moves the arm relative to the current position in carthesian coordinates,
        assuming the gripper is vertical (aligned with the z-axis), pointing down.
        This version returns after the amount of time specified by dt.
        This supports the simplest Gym robotic manipulation environment.
        '''
        self.arm.read()
        pose = self.arm.state.tool_pose()
        pose = Tool.from_xyzrpy(pose.to_xyzrpy() + [dx, dy, dz, 0, 0, dyaw])
        self.hand.move(position=gripper_state, blocking=False)
        self.arm.move(pose, max_speed=max_speed, max_acc=max_acc, blocking=False)
        end = self.arm.state.time() + dt
        while self.arm.state.time() < end and not (self.arm.state.is_done() and self.hand.state.is_done()):
            self.read()

    def read(self):
        self.arm.read()
        self.hand.read()


def connect(config={}):
    '''
    Creates and returns a robot instance with real (hardware) backing.
    '''
    return Robot().connect(use_sim=False, config=config)


def connect_sim(scene_init_fn=None, config={}):
    '''
    Creates and returns a simulated robot instance together with a sim scene manager.
    '''
    r = Robot().connect(use_sim=True, config=config)
    s = SimScene(r, scene_init_fn).connect()
    return r, s
