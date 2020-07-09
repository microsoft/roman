import sys
import numpy as np
import math 
import time
import random
import os
import time
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)
from robot.ur_connection import *
from robot.controllers import *
from robot.types import *

def read_test():
    print(f"Running {__file__}::{read_test.__name__}()")
    con = URConnection()
    con.connect()
    arm_ctrl = ArmController(con)
    cmd = Command().make_read_cmd()
    state = arm_ctrl(cmd)
    print(state.tool_pose())
    con.disconnect()
    print("Passed.")   
    
def move_test():
    print(f"Running {__file__}::{move_test.__name__}()")

    con = URConnection()
    con.connect()
    arm_ctrl = ArmController(con)

    cmd = Command(target_position=Tool(-0.4, -0.4, 0.3,0, math.pi, 0))
    state = arm_ctrl(cmd)
    while not state.is_goal_reached():
        state = arm_ctrl(cmd)

    con.disconnect()
    print("Passed.")    

   
def run():
    #read_test()
    move_test()
   
#env_test()
if __name__ == '__main__':
    run()