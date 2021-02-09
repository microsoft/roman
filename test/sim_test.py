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


def touch(in_proc):
    '''This requires a horizontal surface that the arm can touch.'''
    print(f"Running {__file__}::{touch.__name__}()")
    robot = connect(use_sim = True, in_proc = in_proc)
    home_pose = robot.arm.state.tool_pose().clone()
    below_table = home_pose.clone()
    below_table[2] = -0.2 # lower than the table

    robot.arm.touch(below_table)
    assert robot.arm.state.is_goal_reached()

    # go back
    robot.arm.move(target_position=home_pose, force_low_bound=[-30,-30, -30, -2, -2, -2], force_high_bound = [30, 30, 30, 2, 2, 2])

    robot.disconnect()
    print("Passed.")

def setup_sim(simenv):
    simenv.make_box([0.1,0.1,0.1], [-0.5,-0.1,0.05], color=(0.8,0.2,0.2,1), mass = 0.1)

def pick(in_proc):
    print(f"Running {__file__}::{pick.__name__}()")
    robot = connect(use_sim = True, in_proc = in_proc, sim_init=setup_sim)
    robot.hand.close()
    assert not robot.hand.state.object_detected()
    robot.hand.open()
    assert not robot.hand.state.object_detected()
    home_pose = robot.arm.state.tool_pose().clone()
    grasp_pose = home_pose.clone()
    grasp_pose[2] = 0.05
    robot.arm.move(target_position=grasp_pose)
    robot.hand.close()
    assert robot.hand.state.object_detected()
    robot.arm.move(target_position=home_pose)
    robot.hand.close()
    assert robot.hand.state.object_detected()
    robot.hand.open()
    robot.hand.close()
    assert not robot.hand.state.object_detected()
    
    robot.disconnect()
    print("Passed.")

#############################################################
# Runner
############################################################# 
def run():
    touch(in_proc = True)
    touch(in_proc = False)
    pick(in_proc = True)
    pick(in_proc = True)
    
if __name__ == '__main__':

    run()
    

