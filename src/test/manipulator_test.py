import sys
import numpy as np
import math 
import time
import random
import os
import time
from multiprocessing import Process, Pipe
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)
from roman import *

def arm_move(use_sim):
    print(f"Running {__file__}::{arm_move.__name__}()")
    robot = connect(use_sim = use_sim)
    arm = robot.arm
    position=ur.Joints(0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0)
    arm.move(target_position = position, max_speed=1, max_acc=0.5)
    assert arm.state.joint_positions().allclose(position)

    pose = ur.Tool(-0.4, -0.4, 0.2, 0, math.pi, 0)
    arm.move(target_position=pose, max_speed=1, max_acc=0.5)
    assert arm.state.tool_pose().allclose(pose)

    arm.move(target_position=position, max_speed=1, max_acc=0.5)
    assert arm.state.joint_positions().allclose(position)

    robot.disconnect()
    print("Passed.")

def step(use_sim):
    print(f"Running {__file__}::{step.__name__}()")
    pose = ur.Tool(-0.4, -0.4, 0.2, 0, math.pi, 0)
    robot = connect(use_sim = use_sim)
    d = pose.to_xyzrpy() - robot.arm.state.tool_pose().to_xyzrpy() 
    robot.step(d[0], d[1], d[2], d[5])
    while not robot.arm.state.is_done():
        d = pose.to_xyzrpy() - robot.arm.state.tool_pose().to_xyzrpy() 
        robot.step(d[0], d[1], d[2], d[5])
    assert robot.arm.state.is_goal_reached()

    robot.disconnect()
    print("Passed.")


def hand_move(use_sim):
    print(f"Running {__file__}::{hand_move.__name__}()")
    robot = connect(use_sim = use_sim)
    robot.hand.open()
    assert robot.hand.state.position() == hand.Position.OPENED
    
    robot.hand.close()
    assert robot.hand.state.position() == hand.Position.CLOSED

    robot.hand.open()
    assert robot.hand.state.position() == hand.Position.OPENED

    robot.hand.set_mode(mode=hand.GraspMode.PINCH)
    assert robot.hand.state.position() == hand.Position.OPENED
    assert robot.hand.state.mode() == hand.GraspMode.PINCH
    
    robot.hand.close()
    assert robot.hand.state.position() == hand.Position.CLOSED
    assert robot.hand.state.mode() == hand.GraspMode.PINCH
    
    robot.hand.open()
    assert robot.hand.state.position() == hand.Position.OPENED
    assert robot.hand.state.mode() == hand.GraspMode.PINCH

    robot.hand.set_mode(mode=hand.GraspMode.BASIC)
    assert robot.hand.state.mode() == hand.GraspMode.BASIC
    assert robot.hand.state.position() == hand.Position.OPENED

    robot.disconnect()
    print("Passed.")

def arm_hand_move(use_sim):
    print(f"Running {__file__}::{arm_hand_move.__name__}()")
    robot = connect(use_sim = use_sim)

    robot.hand.open()
    robot.hand.close(blocking=False)
    pose = ur.Tool(-0.4, -0.4, 0.2, 0, math.pi, 0)
    robot.arm.move(target_position=pose, max_speed=1, max_acc=0.5, blocking=False)
    while not robot.arm.state.is_done() or not robot.hand.state.is_done():
        robot.arm.read()
        robot.hand.read()
    assert robot.arm.state.tool_pose().allclose(pose)
    assert robot.hand.state.position() == hand.Position.CLOSED
    
    position = ur.Joints(0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0)
    robot.arm.move(target_position=position, max_speed=1, max_acc=0.5, blocking=False)
    robot.hand.open(blocking=False)
    while not robot.arm.state.is_done() or not robot.hand.state.is_done():
        robot.arm.read()
        robot.hand.read()
    assert robot.arm.state.joint_positions().allclose(position)
    assert robot.hand.state.position() == hand.Position.OPENED
    robot.disconnect()
    print("Passed.")

def arm_touch(use_sim):
    '''This requires a horizontal surface that the arm can touch.'''
    print(f"Running {__file__}::{arm_touch.__name__}()")
    robot = connect(use_sim = use_sim)
    robot.hand.open()
    robot.hand.set_mode(mode=hand.GraspMode.PINCH)
    robot.hand.close()
    home_pos = ur.Joints(0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0)
    robot.arm.move(target_position=home_pos, max_speed=1, max_acc=0.5)
    below_table = robot.arm.state.tool_pose().clone()
    below_table[2] = -0.2 # lower than the table

    robot.arm.touch(below_table, max_speed = 0.1, max_acc=0.1)
    assert robot.arm.state.is_goal_reached()

    # go back
    robot.arm.move(target_position=home_pos, max_speed = 1, max_acc = 1)

    robot.disconnect()
    print("Passed.")

#############################################################
# Runner
############################################################# 
def run(use_sim):
    step(use_sim)
    arm_move(use_sim)
    hand_move(use_sim)
    arm_hand_move(use_sim)
    arm_touch(use_sim)
    
if __name__ == '__main__':
    run(True)
    

