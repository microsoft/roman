
import math
import time
from roman import Robot, SimScene, connect, connect_sim
from roman import Joints, Tool, Position, GraspMode

def arm_read(use_sim, config={}):
    print(f"Running {__file__}::{arm_read.__name__}()")
    robot = Robot(use_sim=use_sim, config=config).connect()
    arm = robot.arm
    position = Joints(0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0)
    arm.read()
    print(arm.state.joint_positions())
    assert not arm.state.is_moving()

    robot.disconnect()
    print("Passed.")

def arm_move(use_sim):
    print(f"Running {__file__}::{arm_move.__name__}()")
    robot = Robot(use_sim=use_sim).connect()
    arm = robot.arm
    position = Joints(0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0)
    arm.move(target_position=position, max_speed=1, max_acc=0.5)
    assert arm.state.joint_positions().allclose(position)
    assert not arm.state.is_moving()

    pose = Tool(-0.4, -0.4, 0.2, 0, math.pi, 0)
    arm.move(target_position=pose, max_speed=1, max_acc=0.5)
    assert arm.state.tool_pose().allclose(pose)
    assert not arm.state.is_moving()

    arm.move(target_position=position, max_speed=1, max_acc=0.5)
    assert arm.state.joint_positions().allclose(position)
    assert not arm.state.is_moving()

    robot.disconnect()
    print("Passed.")


def hand_move(use_sim):
    print(f"Running {__file__}::{hand_move.__name__}()")
    robot = Robot(use_sim=use_sim).connect()
    robot.hand.open()
    assert robot.hand.state.position() == Position.OPENED

    robot.hand.close()
    assert robot.hand.state.position() == Position.CLOSED

    robot.hand.open()
    assert robot.hand.state.position() == Position.OPENED

    robot.hand.set_mode(mode=GraspMode.PINCH)
    assert robot.hand.state.position() == Position.OPENED
    assert robot.hand.state.mode() == GraspMode.PINCH

    robot.hand.close()
    assert robot.hand.state.position() == Position.CLOSED
    assert robot.hand.state.mode() == GraspMode.PINCH

    robot.hand.open()
    assert robot.hand.state.position() == Position.OPENED
    assert robot.hand.state.mode() == GraspMode.PINCH

    robot.hand.set_mode(mode=GraspMode.BASIC)
    assert robot.hand.state.mode() == GraspMode.BASIC
    assert robot.hand.state.position() == Position.OPENED

    robot.disconnect()
    print("Passed.")

def arm_hand_move(use_sim):
    print(f"Running {__file__}::{arm_hand_move.__name__}()")
    robot = Robot(use_sim=use_sim).connect()

    robot.hand.open()
    robot.hand.close(blocking=False)
    pose = Tool(-0.4, -0.4, 0.2, 0, math.pi, 0)
    robot.arm.move(target_position=pose, max_speed=1, max_acc=0.5, blocking=False)
    while not robot.arm.state.is_done() or not robot.hand.state.is_done():
        robot.arm.step()
        robot.hand.step()
    assert robot.arm.state.tool_pose().allclose(pose)
    assert robot.hand.state.position() == Position.CLOSED

    position = Joints(0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0)
    robot.arm.move(target_position=position, max_speed=1, max_acc=0.5, blocking=False)
    robot.hand.open(blocking=False)
    while not robot.arm.state.is_done() or not robot.hand.state.is_done():
        robot.arm.step()
        robot.hand.step()
    assert robot.arm.state.joint_positions().allclose(position)
    assert robot.hand.state.position() == Position.OPENED
    robot.disconnect()
    print("Passed.")

def arm_touch(use_sim, iter=1):
    '''This requires a horizontal surface that the arm can touch.'''
    print(f"Running {__file__}::{arm_touch.__name__}()")
    if use_sim:
        (robot, scene) = connect_sim()
    else:
        robot = connect()
    robot.hand.set_mode(mode=GraspMode.PINCH)
    robot.hand.close()
    
    
    home_pos = Joints(0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0)
    robot.arm.move(target_position=home_pos, max_speed=2, max_acc=1)
    time.sleep(2)
    while iter:
        robot.arm.move(target_position=home_pos, max_speed=2, max_acc=1)
        time.sleep(2)
        below_table = robot.arm.state.tool_pose().clone()
        below_table[2] = -0.2 # lower than the table

        robot.arm.touch(below_table, max_speed=0.1, max_acc=0.1, contact_force_multiplier=5)
        assert robot.arm.state.is_goal_reached()

        # # go back
        robot.arm.move(target_position=home_pos, max_speed=1, max_acc=1)
        iter -= 1

    if use_sim:
        scene.disconnect()
    robot.disconnect()
    print("Passed.")

#############################################################
# Runner
#############################################################
def run(use_sim):
    arm_read(False, {"hand.activate": False})
    arm_move(use_sim)
    #hand_move(use_sim)
    #arm_hand_move(use_sim)
    arm_touch(use_sim=False, iter=1)


if __name__ == '__main__':
    run(True)
