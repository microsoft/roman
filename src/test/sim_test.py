import sys
import numpy as np
import math 
import time
import random
import os
import time
import pybullet as pb
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)
from robot.connection import *
from robot.controllers import *
from robot.types import *
from robot.simenv import SimEnvironment

def env_test():
    env = SimEnvironment()
    env.reset()
    env.update()
    time.sleep(1)

def arm_test():
    print(f"Running {__file__}::{arm_test.__name__}()")
    env = SimEnvironment()
    env.reset()
    env.update()

    con = SimURConnection()
    arm_ctrl = ArmController(con)
    cmd = Command().make_read_cmd()
    state = arm_ctrl(cmd)

    print(state.tool_pose())
    marker_visual_id = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.005, 0.005, 0.005], rgbaColor=[1,0,0,1])
    #pb.createMultiBody(baseVisualShapeIndex=marker_visu
    # al_id, basePosition=state.tool_pose()[:3])
    cmd = Command(target_position=Tool(-0.4, -0.4, 0.3,0, math.pi/2, math.pi))
    pb.createMultiBody(baseVisualShapeIndex=marker_visual_id, basePosition=cmd.target_position()[:3])
    state = arm_ctrl(cmd)
    while not state.is_goal_reached():
        # st = time.time()
        state = arm_ctrl(cmd)
        env.update()
        # latency = time.time()-st
        # leftover = 1/240. - latency
        # if leftover > 0:
        #     time.sleep(leftover)
    
    print("Passed.")    

   
def run():
    env_test()
    arm_test()
   
#env_test()
if __name__ == '__main__':
    run()