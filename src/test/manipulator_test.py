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

def arm_move(real_robot):
    print(f"Running {__file__}::{arm_move.__name__}()")
    m = connect(config={"real_robot":real_robot})

    cmd = ur.Command(target_position=ur.Joints(0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0), max_speed=1, max_acc=0.5)
    m.move_arm(cmd)
    assert m.arm_state.joint_positions().allclose(cmd.target_position())

    cmd = ur.Command(target_position=ur.Tool(-0.4, -0.4, 0.2, 0, math.pi, 0), max_speed=1, max_acc=0.5)
    m.move_arm(cmd)
    assert m.arm_state.tool_pose().allclose(cmd.target_position())

    cmd = ur.Command(target_position=ur.Joints(0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0), max_speed=1, max_acc=0.5)
    m.move_arm(cmd)
    assert m.arm_state.joint_positions().allclose(cmd.target_position())

    m.disconnect()
    print("Passed.")

def hand_move():
    print(f"Running {__file__}::{hand_move.__name__}()")
    m = connect(config={"real_robot":True})
    cmd = hand.Command.open()
    m.move_hand(cmd)
    assert m.hand_state.position_A() == hand.Position.OPENED
    
    cmd = hand.Command.close()
    m.move_hand(cmd)
    assert m.hand_state.position_A() == hand.Position.CLOSED

    cmd = hand.Command.open()
    m.move_hand(cmd)
    assert m.hand_state.position_A() == hand.Position.OPENED

    cmd = hand.Command.open(mode=hand.GraspMode.PINCH)
    m.move_hand(cmd)
    assert m.hand_state.position_A() == hand.Position.OPENED
    assert m.hand_state.mode() == hand.GraspMode.PINCH
    
    cmd = hand.Command.close(mode=hand.GraspMode.PINCH)
    m.move_hand(cmd)
    assert m.hand_state.position_A() == hand.Position.CLOSED
    assert m.hand_state.mode() == hand.GraspMode.PINCH
    
    cmd = hand.Command.open(mode=hand.GraspMode.PINCH)
    m.move_hand(cmd)
    assert m.hand_state.position_A() == hand.Position.OPENED
    assert m.hand_state.mode() == hand.GraspMode.PINCH

    cmd = hand.Command.open(mode=hand.GraspMode.BASIC)
    m.move_hand(cmd)
    assert m.hand_state.mode() == hand.GraspMode.BASIC
    assert m.hand_state.position_A() == hand.Position.OPENED

    m.disconnect()
    print("Passed.")

def arm_hand_move():
    print(f"Running {__file__}::{arm_hand_move.__name__}()")
    m = connect(config={"real_robot":True})
    hand_cmd = hand.Command.open()
    m.move_hand(hand_cmd)

    hand_cmd = hand.Command.close()
    arm_cmd =ur.Command(target_position=ur.Tool(-0.4, -0.4, 0.2, 0, math.pi, 0), max_speed=1, max_acc=0.5)
    m.send_arm_cmd(arm_cmd)
    m.send_hand_cmd(hand_cmd)
    while not m.arm_state.is_done() or not m.hand_state.is_done():
        m.read_arm()
        m.read_hand()
    assert m.arm_state.tool_pose().allclose(arm_cmd.target_position())
    assert m.hand_state.position_A() == hand.Position.CLOSED
    
    arm_cmd = ur.Command(target_position=ur.Joints(0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0), max_speed=1, max_acc=0.5)
    m.send_arm_cmd(arm_cmd)
    hand_cmd = hand.Command.open()
    m.send_hand_cmd(hand_cmd)
    while not m.arm_state.is_done() or not m.hand_state.is_done():
        m.read_arm()
        m.read_hand()
    assert m.arm_state.joint_positions().allclose(arm_cmd.target_position())
    assert m.hand_state.position_A() == hand.Position.OPENED
    m.disconnect()
    print("Passed.")

def arm_touch():
    '''This requires a horizontal surface that the arm can touch.'''
    print(f"Running {__file__}::{arm_move.__name__}()")
    m = connect(config={"real_robot":True})
    m.move_hand(hand.Command.open())
    m.move_hand(hand.Command.open(mode=hand.GraspMode.PINCH))
    m.move_hand(hand.Command.close(mode=hand.GraspMode.PINCH))
    home_pos = ur.Command(target_position=ur.Joints(0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0), max_speed=1, max_acc=0.5)
    m.move_arm(home_pos)
    time.sleep(1)
    below_table = ur.Command(
        target_position=ur.Tool(-0.4, -0.4, -0.2, 0, math.pi, 0), 
        max_speed=0.05, 
        max_acc=0.05, 
        force_low_bound=[-5,-5,-5,-0.5, -0.5, -0.5],
        force_high_bound=[5,5,5,0.5, 0.5, 0.5],
        contact_handling=5, 
        controller = 1)

    while True:
        m.move_arm(below_table)
        m.move_arm(home_pos)
        time.sleep(1)

    m.disconnect()
    print("Passed.")

#############################################################
# Runner
############################################################# 
def run(real_robot = False):
    arm_move(real_robot)
    #arm_touch(real_robot)
    if real_robot:
        hand_move()
        arm_hand_move()

if __name__ == '__main__':
    run(True)
    
