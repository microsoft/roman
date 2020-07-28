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
# Low-level unit tests using the simulated arm. 
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
    cmd.make(kind = UR_CMD_KIND_MOVE_JOINTS_POSITION, target_position=Joints(1,1,1,1,1,1))
    state = State.fromarray(execute_arm_command(cmd, 0))
    #print(state)
    print("Passed.")

def move_arm_test(env):
    print(f"Running {__file__}::{move_arm_test.__name__}()")

    con = SimConnection(env)
    arm_ctrl = BasicController(con)
    cmd = Command()
    state = State()
    arm_ctrl.execute(cmd, state)

    print(state.tool_pose())
    marker_visual_id = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.005, 0.005, 0.005], rgbaColor=[1,0,0,1])
    #pb.createMultiBody(baseVisualShapeIndex=marker_visual_id, basePosition=state.tool_pose()[:3])
    cmd.make(kind = UR_CMD_KIND_MOVE_TOOL_POSE, target_position=Tool(-0.4, -0.4, 0.3,0, math.pi, 0))
    pb.createMultiBody(baseVisualShapeIndex=marker_visual_id, basePosition=cmd.target_position()[:3])
    arm_ctrl.execute(cmd, state)
    while not state.is_goal_reached():
        # st = time.time()
        arm_ctrl.execute(cmd, state)
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