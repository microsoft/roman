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
    cmd = arm.Command(target_position=arm.Tool(-0.4, -0.4, 0.3,0, math.pi/2, math.pi))
    while not m.arm_state.is_done():
        m.cmd_arm(cmd)

    m.disconnect()
    print("Passed.")

def arm_hand_move(real_robot):
    print(f"Running {__file__}::{arm_hand_move.__name__}()")
    m = connect(config={"real_robot":real_robot})
    hand_cmd = hand.Command.open()
    m.cmd_hand(hand_cmd)
    while not m.hand_state.is_done():
        m.cmd_hand(hand_cmd)

    hand_cmd = hand.Command.close()
    arm_cmd = arm.Command(target_position=arm.Tool(-0.4, -0.4, 0.3,0, math.pi/2, math.pi))
    m.cmd_hand(hand_cmd)
    m.cmd_arm(arm_cmd)
    while not m.arm_state.is_done() or not m.hand_state.is_done():
        m.cmd_arm(arm_cmd)
        m.cmd_hand(hand_cmd)

    m.disconnect()
    print("Passed.")
    
def run(real_robot = False):
    arm_move(real_robot)
    if real_robot:
        arm_hand_move(real_robot)
#env_test()
if __name__ == '__main__':
    run(True)
