import os
import numpy as np
import threading
import time
from .rq import hand
from . import rq
from .ur import arm
from . import ur
from . import server

class Robot:
    '''
    Combines the manipulator components (arm, hand, FT and tactile sensors).
    '''
    def connect(self, config):
        self.host = server.start(config)
        self.arm = arm.Arm(self.host.arm)
        self.hand = hand.Hand(self.host.hand)
        self.arm.read()
        self.hand.read()

    def disconnect(self):
        self.host.stop()

    def move_simple(self, dx, dy, dz, dyaw, gripper_state=hand.Position.OPENED, max_speed = 0.5):
        '''
        Moves the arm relative to the current position in carthesian coordinates,
        assuming the gripper is vertical (aligned with the z-axis), pointing down.
        This supports the simplest Gym robotic manipulation environment.
        '''
        self.arm.read()
        pose = self.arm.state.tool_pose()
        pose = arm.Tool.from_xyzrpy(pose.to_xyzrpy() + [dx,dy, dz,0,0, dyaw])
        self.arm.move(pose, max_speed = max_speed)
        self.hand.move(position = gripper_state)

    def step(self, dx, dy, dz, dyaw, gripper_state=hand.Position.OPENED, max_speed = 0.5, max_acc=0.5, dt = 0.2):
        '''
        Moves the arm relative to the current position in carthesian coordinates,
        assuming the gripper is vertical (aligned with the z-axis), pointing down.
        This version returns after the amount of time specified by dt.
        This supports the simplest Gym robotic manipulation environment.
        '''
        self.arm.read()
        pose = self.arm.state.tool_pose()
        pose = arm.Tool.from_xyzrpy(pose.to_xyzrpy() + [dx,dy, dz,0,0, dyaw])
        self.hand.move(position = gripper_state, blocking = False)
        self.arm.move(pose, max_speed = max_speed, max_acc=max_acc, blocking = False)
        end = self.arm.state.time() + dt
        while self.arm.state.time() < end and not (self.arm.state.is_done() and self.hand.state.is_done()):
            self.read()

    def read(self):
        self.arm.read()
        self.hand.read()

def connect(use_sim = True, in_proc = False, sim_init = None):
    '''
    Creates a robot instance with either sim or real (hardware) backing.
    By default, sim runs in-proc and real runs out-of-proc.
    Note that when running in-proc the async methods behave differently (since there's no server to execute them)
    and need to be called in a tight loop.
    '''
    m = Robot()
    m.connect(config={"use_sim":use_sim , "in_proc":in_proc, "sim.init":sim_init})
    return m


