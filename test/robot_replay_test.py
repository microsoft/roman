import math
import random
import time
from roman import Robot, Tool, Joints, NO_FORCE_LIMIT
from roman.sim.simenv import SimEnv

class Writer():
    def __init__(self, interval=0.033):
        self.frames = []
        self.last_time = 0
        self.interval = interval

    def __call__(self, arm_state, hand_state, arm_cmd, hand_cmd):
        if arm_state.time() - self.last_time < self.interval:
            return
        self.frames.append((arm_state.clone(), hand_state.clone(), arm_cmd.clone(), hand_cmd.clone()))
        self.last_time = arm_state.time()

def replay_speed(use_sim, iterations=1):
    print(f"Running {__file__}::{replay_speed.__name__}()")
    start_joints = Joints(0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0)
    if not use_sim:
        robot = Robot(use_sim=use_sim, config={"hand.activate": False}).connect()
        robot.move(start_joints, max_speed=1, max_acc=1)
        robot.disconnect()
    for iter in range(iterations):
        writer = Writer()
        robot = Robot(use_sim=use_sim, writer=writer, config={"hand.activate": False}).connect()
        home = robot.tool_pose
        target = Tool.from_xyzrpy(home.to_xyzrpy() + [0.1 * random.random(), 0.1 * random.random(), -0.1 * random.random(), 0.25 * random.random(), 0.25 * random.random(), 0.25 * random.random()])
        start = robot.last_state()[0].time()
        robot.move(target)
        robot.stop()
        robot.stop()
        assert(not robot.is_moving())
        print(len(writer.frames))
        print(robot.last_state()[0].time() - start)
        robot.disconnect()

        robot = Robot(use_sim=use_sim, config={"hand.activate": False}).connect()
        robot.move(home, max_speed=1, max_acc=1)
        robot.stop()
        start = robot.last_state()[0].time()
        initial = writer.frames[0][0].time()
        for i in range(len(writer.frames) - 1):
            timeout = writer.frames[i + 1][0].time() - writer.frames[i][0].time()
            target_speeds = writer.frames[i + 1][0].joint_speeds()
            max_acc = writer.frames[i][2].max_acceleration()
            while robot.last_state()[0].time() - start < writer.frames[i + 1][0].time() - initial:
                robot.move(target_speeds, max_acc=max_acc, timeout=0)
            if use_sim:
                assert(robot.arm.state.joint_speeds().allclose(target_speeds))

        print(robot.last_state()[0].time() - start)
        assert(robot.tool_pose.allclose(target, position_tolerance=0.005, rotation_tolerance=0.05))
        assert(not robot.is_moving())
        robot.move(home, max_speed=1, max_acc=1)
        robot.disconnect()

    print("Passed.")

def replay_pose(use_sim, iterations=1):
    print(f"Running {__file__}::{replay_pose.__name__}()")
    # init
    max_speed = 0.5
    max_acc = 0.5
    start_joints = Joints(0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0)
    if not use_sim:
        robot = Robot(use_sim=use_sim, config={"hand.activate": False})
        robot.connect()
        robot.move(start_joints, max_speed=1, max_acc=1)
        robot.disconnect()
    # record
    writer = Writer(interval=0.033)
    robot = Robot(use_sim=use_sim, writer=writer, config={"hand.activate": False})
    robot.connect()
    home = robot.tool_pose
    start = robot.last_state()[0].time()
    for iter in range(iterations):
        target = Tool.from_xyzrpy(home.to_xyzrpy() + [0.05 * random.random(), 0.05 * random.random(), -0.05 * random.random(), 0.25 * random.random(), 0.25 * random.random(), 0.25 * random.random()])
        robot.move(target, max_speed=max_speed, max_acc=max_acc, max_final_speed=max_speed)
        assert(robot.is_moving())
    robot.stop()
    robot.stop()
    assert(not robot.is_moving())
    print(robot.last_state()[0].time() - start)
    robot.disconnect()

    # replay
    robot = Robot(use_sim=use_sim, config={"hand.activate": False})
    robot.connect()
    robot.move(home, max_speed=1, max_acc=1)
    robot.stop()
    print(f"replaying {len(writer.frames)} steps")
    robot.read()
    deadline = robot.arm.state.time()
    for i in range(len(writer.frames) - 1):
        timeout = writer.frames[i + 1][0].time() - writer.frames[i][0].time()
        target_pose = writer.frames[i + 1][0].tool_pose()
        # while not robot.arm.state.is_goal_reached():
        #     robot.move(target_pose, max_speed=1, max_acc=1, timeout=0)
        robot.move_rt(target_pose, timeout, max_speed=max_speed, max_acc=max_acc)
        #robot.move(target_pose, max_speed=1.2*max_speed, max_acc=1.2*max_acc, timeout=timeout, max_final_speed=2)
        # if use_sim:
        #     assert(robot.arm.state.joint_speeds().allclose(target_speeds))
    print(robot.last_state()[0].time() - start)
    assert(robot.tool_pose.allclose(target, position_tolerance=0.005, rotation_tolerance=0.05))
    assert(not robot.is_moving())
    robot.disconnect()
    print("Passed.")


def run(use_sim):
    replay_pose(use_sim, 5)
    #replay_speed(use_sim, 1)

if __name__ == '__main__':
    run(use_sim=False)

