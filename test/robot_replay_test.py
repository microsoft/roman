import math
import random
import time
from roman import Robot, Tool
from roman.sim.simenv import SimEnv

class Writer():
    def __init__(self):
        self.frames = []
        self.last_time = 0

    def __call__(self, arm_state, hand_state, arm_cmd, hand_cmd):
        if arm_state.time() - self.last_time < 0.033:
            return
        self.frames.append((arm_state.clone(), hand_state.clone(), arm_cmd.clone(), hand_cmd.clone()))
        self.last_time = arm_state.time()

def replay_speed(use_sim, iterations=1):
    print(f"Running {__file__}::{replay_speed.__name__}()")
    for iter in range(iterations):
        writer = Writer()
        robot = Robot(use_sim=use_sim, writer=writer).connect()
        home = robot.tool_pose
        target = Tool.from_xyzrpy(home.to_xyzrpy() + [-0.2 * random.random(), -0.2 * random.random(), 0.2 * random.random(), 0.5 * random.random(), 0.5 * random.random(), 0.5 * random.random()])
        start = robot.last_state()[0].time()
        robot.move(target)
        robot.stop()
        robot.stop()
        print(len(writer.frames))
        print(robot.last_state()[0].time() - start)

        robot.disconnect()
        robot = Robot(use_sim=use_sim).connect()
        start = robot.last_state()[0].time()
        initial = writer.frames[0][0].time()
        for i in range(len(writer.frames) - 1):
            timeout = writer.frames[i + 1][0].time() - writer.frames[i][0].time()
            target_speeds = writer.frames[i + 1][0].joint_speeds()

            while robot.last_state()[0].time() - start < writer.frames[i + 1][0].time() - initial:
                robot.move(target_speeds, timeout=0)
            assert(robot.joint_speeds.allclose(target_speeds))

        print(robot.last_state()[0].time() - start)
        assert(robot.tool_pose.allclose(target, position_tolerance=0.002))
        assert(not robot.is_moving())
        robot.disconnect()

    print("Passed.")

def run(use_sim):
    replay_speed(use_sim)

if __name__ == '__main__':
    run(use_sim=True)

