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
from roman.arm.ur_connection import *
from roman.arm.sim_connection import *
from roman.arm.controllers import *
from roman.arm.types import *
from roman.sim.simenv import *

def read_test(con):
    print(f"Running {__file__}::{read_test.__name__}()")
    con.connect()
    arm_ctrl = ArmController(con)
    cmd = Command().make_read_cmd()
    state = arm_ctrl(cmd)
    print(state.tool_pose())
    con.disconnect()
    print("Passed.")   
    
def move_test(con):
    print(f"Running {__file__}::{move_test.__name__}()")
    con.connect()
    arm_ctrl = ArmController(con)

    #cmd = Command(target_position=Tool(-0.4, -0.4, 0.3,0, math.pi, 0))
    cmd = Command(target_position=Tool(-0.4, -0.4, 0.3,0, math.pi/2, math.pi))
    state = arm_ctrl(cmd)
    while not state.is_goal_reached():
        state = arm_ctrl(cmd)

    con.disconnect()
    print("Passed.")    

   
def run(real_robot = False):
    if real_robot:
        read_test(URConnection())
        move_test(URConnection())
    else:
        env = SimEnvironment()
        env.reset()
        read_test(SimURConnection(env))
        move_test(SimURConnection(env))
        env.disconnect()
   
#env_test()
if __name__ == '__main__':
    run()