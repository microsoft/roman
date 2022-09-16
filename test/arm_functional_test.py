import math
import numpy as np
from roman import ur
from roman.sim.simenv import SimEnv

#############################################################
# Arm unit tests that bypass the manipulator server.

# These tests can run on either sim or real arm.
#############################################################
def read_test(con):
    print(f"Running {__file__}::{read_test.__name__}()")
    arm_ctrl = ur.BasicController(con)
    cmd = ur.Command()
    state = ur.State()
    arm_ctrl.execute(cmd, state)
    print("Tool pose:" + str(state.tool_pose()))
    print("Joint positions:" + str(state.joint_positions()))
    print("Passed.")

def kin_test(con):
    arm_ctrl = ur.ArmController(con)
    arm = ur.Arm(arm_ctrl)
    target_position=ur.Joints(-math.pi / 4, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 4, -math.pi / 4)
    arm.move(target_position, max_speed=0, max_acc=0, timeout=0)
    target_pose = arm.state.target_tool_pose()
    arm.move(target_pose, max_speed=0, max_acc=0, timeout=0)
    target_position2 = arm.state.target_joint_positions()
    assert(target_position == target_pose)

def move_test(con):
    print(f"Running {__file__}::{move_test.__name__}()")

    variants = [(ur.BasicController, 2),
                (ur.ArmController, 1)]
    for variant in variants:
        arm_ctrl = variant[0](con)
        arm = ur.Arm(arm_ctrl)
        ms = 1
        ma = 0.5
        fmin = np.array(ur.UR_DEFAULT_FORCE_LOW_BOUND) * variant[1]
        fmax = np.array(ur.UR_DEFAULT_FORCE_HIGH_BOUND) * variant[1]
        arm.move(target_position=ur.Joints(0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0), max_speed=ms, max_acc=ma, force_low_bound=fmin, force_high_bound=fmax)
        assert arm.state.is_goal_reached()
        assert not arm.state.is_moving()
        arm.move(target_position=ur.Tool(-0.4, -0.4, 0.2, 0, math.pi, 0), max_speed=ms, max_acc=ma, force_low_bound=fmin, force_high_bound=fmax)
        assert arm.state.is_goal_reached()
        assert not arm.state.is_moving()
        arm.move(target_position=ur.Joints(0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0), max_speed=ms, max_acc=ma, force_low_bound=fmin, force_high_bound=fmax)
        assert arm.state.is_goal_reached()
        assert not arm.state.is_moving()
        arm.move(target_position=ur.Tool(-0.2, -0.2, 0.3, 0, math.pi, math.pi / 2), max_speed=ms, max_acc=ma, force_low_bound=fmin, force_high_bound=fmax)
        assert arm.state.is_goal_reached()
        assert not arm.state.is_moving()
        arm.move(target_position=ur.Joints(0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0), max_speed=ms, max_acc=ma, force_low_bound=fmin, force_high_bound=fmax)
        assert arm.state.is_goal_reached()
        assert not arm.state.is_moving()
        print("Tool pose:" + str(arm.state.tool_pose()))
        print("Joint positions:" + str(arm.state.joint_positions()))

    print("Passed.")

def move_test2(con):
    print(f"Running {__file__}::{move_test2.__name__}()")
    variants = [(ur.BasicController, 2),
                (ur.ArmController, 1)]
    for variant in variants:
        arm_ctrl = variant[0](con)
        arm = ur.Arm(arm_ctrl)
        ms = 1
        ma = 0.5
        fmin = np.array(ur.UR_DEFAULT_FORCE_LOW_BOUND) * variant[1]
        fmax = np.array(ur.UR_DEFAULT_FORCE_HIGH_BOUND) * variant[1]

        arm.move(target_position=ur.Joints(0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0), force_low_bound=fmin, force_high_bound=fmax)
        assert arm.state.is_goal_reached()
        assert not arm.state.is_moving()

        home = arm.state.tool_pose() + 0
        arm.move(target_position=home, force_low_bound=fmin, force_high_bound=fmax)
        assert arm.state.is_goal_reached()
        assert not arm.state.is_moving()

        ms = 1
        ma = 0.5

        next = home + [0, 0, 0.1, 0, 0, math.pi / 2]
        #next = home + [0, 0, 0.25, 0, 0, 0]
        arm.move(target_position=next, max_speed=ms, max_acc=ma, force_low_bound=fmin, force_high_bound=fmax)
        assert arm.state.is_goal_reached()
        assert not arm.state.is_moving()

        down = home + [0, 0, -0.1, 0, 1, 0]
        arm.move(target_position=down, max_speed=ms, max_acc=ma, force_low_bound=fmin, force_high_bound=fmax)
        assert arm.state.is_goal_reached()
        assert not arm.state.is_moving()

        arm.move(target_position=next, max_speed=ms, max_acc=ma, force_low_bound=fmin, force_high_bound=fmax)
        assert arm.state.is_goal_reached()
        assert not arm.state.is_moving()

        arm.move(target_position=down, max_speed=ms, max_acc=ma, force_low_bound=fmin, force_high_bound=fmax)
        assert arm.state.is_goal_reached()
        assert not arm.state.is_moving()

        arm.move(target_position=home, max_speed=ms, max_acc=ma, force_low_bound=fmin, force_high_bound=fmax)
        assert arm.state.is_goal_reached()
        assert not arm.state.is_moving()

        print("Tool pose:" + str(arm.state.tool_pose()))
        print("Joint positions:" + str(arm.state.joint_positions()))

    print("Passed.")

#############################################################
# Runner
#############################################################
def run(use_sim):
    if not use_sim:
        con = ur.Connection()
    else:
        env = SimEnv()
        env.connect()
        con = ur.SimConnection(env)

    con.connect()
    read_test(con)
    kin_test(con)
    move_test(con)
    move_test2(con)
    con.disconnect()

    if use_sim:
        env.disconnect()


if __name__ == '__main__':
    run(use_sim=True)
