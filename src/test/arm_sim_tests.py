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
from roman.ur import *
from roman.sim.ur_rq3 import SimEnv
from roman.ur.scripts.interface import *

#############################################################
# Unit tests that bypass the manipulation server 
#############################################################
def get_arm_state_test(env):
    print(f"Running {__file__}::{get_arm_state_test.__name__}()")
    state = State.fromarray(get_arm_state())
    #print(state)
    print("Passed.")

def execute_arm_command_test(env):
    print(f"Running {__file__}::{execute_arm_command_test.__name__}()")
    cmd = Command()
    state = State.fromarray(execute_arm_command(cmd, 0))
    cmd = Command(target_position=Joints(1,1,1,1,1,1))
    state = State.fromarray(execute_arm_command(cmd, 0))
    #print(state)
    print("Passed.")

def move_arm_test(env):
    print(f"Running {__file__}::{move_arm_test.__name__}()")

    con = SimConnection(env)
    arm_ctrl = ArmController(con)
    cmd = Command().read()
    state = arm_ctrl(cmd)

    print(state.tool_pose())
    marker_visual_id = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.005, 0.005, 0.005], rgbaColor=[1,0,0,1])
    #pb.createMultiBody(baseVisualShapeIndex=marker_visual_id, basePosition=state.tool_pose()[:3])
    cmd = Command(target_position=Tool(-0.4, -0.4, 0.3,0, math.pi, 0))
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


#############################################################
# Runner
#############################################################
def run():
    env = SimEnv()
    env.reset()

    get_arm_state_test(env)
    execute_arm_command_test(env)
    move_arm_test(env)

    env.disconnect()
   
if __name__ == '__main__':
    run()