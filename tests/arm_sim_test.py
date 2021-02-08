import math 
import os
import random
import sys
import time

import numpy as np
import pybullet as pb

rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)

from roman import ur
from roman.sim.ur_rq3 import SimEnv
from roman.ur.realtime import constants
from roman.ur.realtime import interface
from roman.ur.realtime import urlib

#############################################################
# Low-level unit tests using the simulated arm. 
#############################################################

def pose_op_test():
    print(f"Running {__file__}::{pose_op_test.__name__}()")

    x = urlib.pose_sub([1,1,1,1,1,1], [2,2,2,2,2,2])
    assert np.allclose(x, [-1,-1, -1, -1, -1, -1])
    x = urlib.interpolate_pose([1,1,1,1,1,1], [2,2,2,2,2,2], 0.2)
    assert np.allclose(x, [1.2, 1.2, 1.2, 1.2, 1.2, 1.2])
    x = urlib.interpolate_pose([1,1,1,1,1,1], [0,0,0,0,0,0], 0.1)
    assert np.allclose(x, [0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    x = urlib.interpolate_pose([1,2,3,4,5,6], [0,0,0,0,0,0], 0.1)
    assert np.allclose(x, [0.9,1.8,2.7, 1.02227, 1.27784, 1.53341]) # copied from UR script results

    x = urlib.interpolate_pose([-4.86900002e-01,-1.09150000e-01, 1.71858996e-01, -2.22144147e+00, -2.22144147e+00, 5.98088608e-09], [-0.4869000017642975, -0.1091499999165535, 0.2718589961528778, -2.221441466386838, -2.221441466386838, 1.5707963327757826], 0.95)
    assert np.allclose(x, [-0.4869 , -0.10915 , 0.266859 , 1.78370568, 1.78370568, -1.18747401])

    print("Passed.")

def get_arm_state_test(env):
    print(f"Running {__file__}::{get_arm_state_test.__name__}()")
    urlib.sim = env
    ur.arm.State.fromarray(interface.get_arm_state())
    #print(state)
    print("Passed.")

def execute_arm_command_test(env):
    print(f"Running {__file__}::{execute_arm_command_test.__name__}()")
    urlib.sim = env
    cmd = ur.arm.Command()
    ur.arm.State.fromarray(interface.execute_arm_command(cmd, 0))
    cmd.make(kind = constants.UR_CMD_KIND_MOVE_JOINTS_POSITION, target=ur.arm.Joints(1,1,1,1,1,1))
    ur.arm.State.fromarray(interface.execute_arm_command(cmd, 0))
    #print(state)
    print("Passed.")

def move_arm_test(env):
    print(f"Running {__file__}::{move_arm_test.__name__}()")

    con = ur.sim_connection.SimConnection(env)
    arm_ctrl = ur.controllers.BasicController(con)
    cmd = ur.arm.Command()
    state = ur.arm.State()
    arm_ctrl.execute(cmd, state)

    print(state.tool_pose())
    marker_visual_id = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.005, 0.005, 0.005], rgbaColor=[1,0,0,1])
    #pb.createMultiBody(baseVisualShapeIndex=marker_visual_id, basePosition=state.tool_pose()[:3])
    cmd.make(kind = constants.UR_CMD_KIND_MOVE_TOOL_POSE, target=ur.arm.Tool(-0.4, -0.4, 0.3,0, math.pi, 0))
    pb.createMultiBody(baseVisualShapeIndex=marker_visual_id, basePosition=cmd.target()[:3])
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
    pose_op_test()

    env = SimEnv()
    env.connect()
    get_arm_state_test(env)
    execute_arm_command_test(env)
    move_arm_test(env)
    env.disconnect()
   
if __name__ == '__main__':
    run()