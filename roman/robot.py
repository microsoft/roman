import os
import numpy as np 
import threading
import time
from .rq import hand
from . import rq
from .ur import arm
from .ur.arm import Tool
from . import ur
from . import server 
from .sim.ur_rq3 import SimEnv

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
        self.visual_obs_server = self.host.get_visual_obs_server()
        self.world_state_server = self.host.get_world_state_server()

    def disconnect(self):
        self.host.stop()

    def move_simple(self, dx, dy, dz, dyaw, gripper_state=hand.Position.OPENED, touch=False, max_speed = 0.5):
        '''
        Moves the arm relative to the current position in carthesian coordinates, 
        assuming the gripper is vertical (aligned with the z-axis), pointing down.
        This supports the simplest Gym robotic manipulation environment.
        '''
        self.arm.read()
        pose = self.arm.state.tool_pose()
        pose = arm.Tool.from_xyzrpy(pose.to_xyzrpy() + [dx, dy, dz, 0, 0, dyaw])

        if touch:
            self.arm.touch(pose, max_speed = max_speed)
        else:
            self.arm.move(pose, max_speed = max_speed)

        self.arm.move(pose, max_speed = max_speed)
        self.hand.move(position = gripper_state)

    def step(self, dx, dy, dz, dyaw, gripper_state=hand.Position.OPENED, touch=False, max_speed=0.5, max_acc=0.5, blocking=True, dt=0.2):
        '''
        Moves the arm relative to the current position in carthesian coordinates, 
        assuming the gripper is vertical (aligned with the z-axis), pointing down.
        This version returns after the amount of time specified by dt.
        This supports the simplest Gym robotic manipulation environment.
        '''
        self.arm.read()
        pose = self.arm.state.tool_pose()
        pose = pose.to_xyzrpy() + [dx, dy, dz, 0, 0, dyaw]
        x, y, z, _, _, yaw = pose
        arm_state, gripper_state = self.step_absolute(x, y, z, yaw, gripper_state=gripper_state, touch=touch, max_speed=max_speed, max_acc=max_acc, blocking=blocking, dt = dt)
        return arm_state, gripper_state

    
    def step_absolute(self, x, y, z, yaw, gripper_state=hand.Position.OPENED, touch=False, max_speed=0.5, max_acc=0.5, blocking=True, dt=0.2):
        '''
        Moves the arm to the specified absolute position in carthesian coordinates, 
        assuming the gripper is vertical (aligned with the z-axis), pointing down.
        This version returns after the amount of time specified by dt.
        This supports the simplest Gym robotic manipulation environment.
        '''
        pose = self.arm.state.tool_pose().to_xyzrpy()
        pose = Tool.from_xyzrpy([x, y, z, pose[Tool.E1], pose[Tool.E2], yaw])
        self.hand.move(position=gripper_state, blocking=blocking)

        if touch:
            self.arm.touch(pose, max_speed=max_speed, max_acc=max_acc, blocking=blocking)
        else:
            self.arm.move(pose, max_speed=max_speed, max_acc=max_acc, blocking=blocking)

        arm_state, gripper_state = self.read()

        if not blocking:
            end = self.arm.state.time() + dt
            while self.arm.state.time() < end and not (self.arm.state.is_done() and self.hand.state.is_done()):
                arm_state, gripper_state = self.read()

        return arm_state, gripper_state

    def read(self):
        arm_state = self.arm.read()
        arm_state = arm_state.tool_pose().to_xyzrpy()
        hand_state = self.hand.read()
        gripper_state = hand_state.grasp_size()
        return arm_state, gripper_state

def connect(use_sim = True, use_gui = True, in_proc = False, sim_init = None):
    '''
    Creates a robot instance with either sim or real (hardware) backing. 
    By default, sim runs in-proc and real runs out-of-proc.
    Note that when running in-proc the async methods behave differently (since there's no server to execute them)
    and need to be called in a tight loop.
    '''
    m = Robot()
    m.connect(config={"use_sim":use_sim, "use_gui":use_gui, "in_proc":in_proc, "sim.init":sim_init})
    return m


