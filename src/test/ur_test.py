################################################################
# This test requires a physical robot
################################################################
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
from roman.sim.ur import SimEnv 

def read_test(con):
    print(f"Running {__file__}::{read_test.__name__}()")
    con.connect()
    arm_ctrl = ur.ArmController(con)
    cmd = ur.Command().read()
    state = arm_ctrl(cmd)
    print(state.tool_pose())
    con.disconnect()
    print("Passed.")   
    
def move_test(con):
    print(f"Running {__file__}::{move_test.__name__}()")
    con.connect()
    arm_ctrl = ur.ArmController(con)

    #cmd = Command(target_position=Tool(-0.4, -0.4, 0.3,0, math.pi, 0))
    cmd = ur.Command(target_position=Tool(-0.4, -0.4, 0.3,0, math.pi/2, math.pi))
    state = arm_ctrl(cmd)
    while not state.is_goal_reached():
        state = arm_ctrl(cmd)

    con.disconnect()
    print("Passed.")    

   
def run(real_robot = False):
    if real_robot:
        read_test(ur.Connection())
        move_test(ur.Connection())
    else:
        env = SimEnv()
        env.reset()
        read_test(ur.SimConnection(env))
        move_test(ur.SimConnection(env))
        env.disconnect()
   
#env_test()
if __name__ == '__main__':
    run(True)