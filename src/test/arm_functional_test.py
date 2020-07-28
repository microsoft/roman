import sys
import numpy as np
import math 
import time
import random
import os
import time
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)
from roman import ur
from roman.sim.ur_rq3 import SimEnv 

#############################################################
# Arm unit tests that bypass the manipulator server.
# These tests can run on either sim or real arm.
#############################################################
def read_test(con):
    print(f"Running {__file__}::{read_test.__name__}()")
    con.connect()
    arm_ctrl = ur.BasicController(con)
    cmd = ur.Command()
    state = ur.State()
    arm_ctrl.execute(cmd, state)
    print("Tool pose:" + str(state.tool_pose()))
    print("Joint positions:" + str(state.joint_positions()))
    con.disconnect()
    print("Passed.")   
    
def move_test(con):
    print(f"Running {__file__}::{move_test.__name__}()")
    con.connect()
    arm_ctrl = ur.BasicController(con)

    arm = ur.Arm(arm_ctrl)
    arm.move(target_position=ur.Joints(0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0))
    assert arm.state.is_goal_reached()
    arm.move(target_position=ur.Tool(-0.4, -0.4, 0.2, 0, math.pi, 0))
    assert arm.state.is_goal_reached()
    arm.move(target_position=ur.Joints(0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0))
    assert arm.state.is_goal_reached()

    print("Tool pose:" + str(arm.state.tool_pose()))
    print("Joint positions:" + str(arm.state.joint_positions()))
    con.disconnect()
    print("Passed.")    

#############################################################
# Runner
#############################################################
def run(real_robot = False):
    if real_robot:
        #read_test(ur.Connection())
        move_test(ur.Connection())
    else:
        env = SimEnv()
        env.reset()
        read_test(ur.SimConnection(env))
        move_test(ur.SimConnection(env))
        env.disconnect()

if __name__ == '__main__':
    run(True)