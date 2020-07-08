import sys
import numpy as np
import math 
import time
import random
import os
import socket
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)
print(rootdir)
import robot

def vec_test():
    p = robot.tool_vec(0,0,0,0,0,0)
    print(p)

def start_arm():
    arm = robot.UR5Arm()
    arm.connect()
    time.sleep(1)
    arm.disconnect()

def basic_com_test():
    arm = robot.UR5Arm()
    arm.connect()

    # just attempt to move to the current location (should be a no-op)
    time.sleep(1)
    arm.read()
    p = arm.tool_pose()
    arm.movep(p)
    time.sleep(1)
    arm.disconnect()

def move_to_ready_test():
    arm = robot.UR5Arm()
    arm.connect()
    arm.movep(robot.ur_arm.KnownPose.READY)
    arm.disconnect()

def move_test():
    arm = robot.UR5Arm()
    arm.connect()
    tp = arm.tool_pose()
    for i in range(10): 
        arm.movep(robot.ur_arm.KnownPose.READY)
        print(i)
        arm.movep(tp)
        print(i)
    arm.disconnect()

def sim_arm():
    arm = robot.SimArm()
    print(arm.joint_positions())
    print(arm.joint_speeds())
    arm.connect()

def run():
    start_arm()
    basic_com_test()
    move_to_ready_test()
    move_test()
    vec_test()
    sim_arm()
   
#env_test()
if __name__ == '__main__':
    run()
