import sys
import numpy as np
import math 
import time
import random
import os
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)
from robot.connection import *
from robot.controllers import *
from robot.types import *

def chain_test():
    print(f"Running {__file__}::{chain_test.__name__}()")
    con = Connection()
    arm_ctrl = ArmController(con)
    force_calib_ctrl = EMAForceCalibrator(arm_ctrl)
    touch_ctrl = TouchController(force_calib_ctrl)

    cmd = Command(target_position=Tool(0.1,0.1,0.1,0,0,0), force_low_bound=Tool(-1,-1,-1,-1,-1,-1), force_high_bound=Tool(1,1,1,1,1,1))
    state = State()
    state[:] = touch_ctrl(cmd)
    print("Passed.")

def arm_controller_test():
    print(f"Running {__file__}::{arm_controller_test.__name__}()")
    arm_ctrl = ArmController(Connection())
    cmd = Command(target_position=Tool(1,1,1,0,0,0))
    state = arm_ctrl(cmd)
    assert not state.is_goal_reached()
    cmd = Command(target_position=Tool(0,0,0,0,0,0))
    state = arm_ctrl(cmd)
    assert state.is_goal_reached()

    cmd = Command(target_position=Joints(1,1,1,0,0,0))
    state = arm_ctrl(cmd)
    assert not state.is_goal_reached()
    cmd = Command(target_position=Joints(0,0,0,0,0,0))
    state = arm_ctrl(cmd)
    assert state.is_goal_reached()

    cmd = Command(target_speed=Joints(1,1,1,0,0,0))
    state = arm_ctrl(cmd)
    assert not state.is_goal_reached()
    cmd = Command(target_speed=Joints(0,0,0,0,0,0))
    state = arm_ctrl(cmd)
    assert state.is_goal_reached()
    
    print("Passed.")

def force_calibration_controller_test():
    print(f"Running {__file__}::{force_calibration_controller_test.__name__}()")
    arm_state = State()
    alpha=0.2
    ctrl = EMAForceCalibrator(lambda cmd: (arm_state), alpha=alpha)
    cmd = Command()
    state = ctrl(cmd)
    assert np.all(ctrl.force_average.array == 0)
    
    arm_state.sensor_force()[:] = [2,2,2,2,2,2]
    cmd = Command()
    state = ctrl(cmd)
    assert np.all(ctrl.force_average.array != 0)
    assert state.sensor_force().allclose(np.array([2,2,2,2,2,2])*(1-alpha), 0.001)

    for i in range(100):
        state = ctrl(cmd)
    assert ctrl.force_average.allclose([2,2,2,2,2,2], 0.001)
    assert state.sensor_force().allclose([0,0,0,0,0,0], 0.001)

    arm_state.sensor_force()[:] = [0,0,0,0,0,0]
    state = ctrl(cmd)
    assert state.sensor_force().allclose(np.array([-2,-2,-2,-2,-2,-2])*(1-alpha), 0.001)

    for i in range(100):
        state = ctrl(cmd)
    assert ctrl.force_average.allclose([0,0,0,0,0,0], 0.001)
    assert state.sensor_force().allclose([0,0,0,0,0,0], 0.001)
   
    print("Passed.")

def touch_controller_test():
    print(f"Running {__file__}::{touch_controller_test.__name__}()")
    arm_state = State()
    arm_state[State._STATUS] = State._STATUS_FLAG_GOAL_REACHED
    cmd = Command(target_position=Joints(0,0,0,0,0,0))
    ctrl = TouchController(lambda cmd: (arm_state), validation_count=3)
    state = ctrl(cmd)
    assert not state.is_goal_reached()
    assert state.is_done()

    arm_state[State._STATUS] = State._STATUS_FLAG_CONTACT
    cmd = Command(target_position=Joints(1,1,1,1,1,1))
    state = ctrl(cmd)
    assert not state.is_goal_reached()
    assert not state.is_done()
    state = ctrl(cmd)
    state = ctrl(cmd)
    state = ctrl(cmd)
    assert state.is_goal_reached()
    assert state.is_done()

    print("Passed.")

def run():
    chain_test()
    arm_controller_test()
    force_calibration_controller_test()
    touch_controller_test()
   
#env_test()
if __name__ == '__main__':
    run()
