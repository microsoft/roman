# this deals with the case when RoMan is not installed as a package
import os
import cv2

from roman import connect_sim, GraspMode
import math
import time

cubePoses = [[-0.15, -0.3, 0.025],
             [-0.35, -0.1, 0.025],
             [-0.55, -0.3, 0.025]]
cubeSize = 0.05

if __name__ == '__main__':
    # start simulation
    #robot = connect(use_sim=True, sim_init=setup_sim)
    (robot, scene) = connect_sim()
    scene.make_box([cubeSize] * 3, cubePoses[0], color=(0.8, 0.2, 0.2, 1), mass=0.1)
    scene.make_box([cubeSize] * 3, cubePoses[1], color=(0.2, 0.8, 0.2, 1), mass=0.1)
    scene.make_box([cubeSize] * 3, cubePoses[2], color=(0.2, 0.2, 0.8, 1), mass=0.1)

    home_pose = robot.arm.state.tool_pose().clone()
    stack_pose = home_pose.clone()
    grasp_depth = 0.01
    robot.hand.move(128)
    robot.hand.set_mode(GraspMode.PINCH)

    # stack the cubes
    for i in range(3):
        pose = home_pose.clone()
        pose[:3] = cubePoses[i]
        pose[2] = 0.1 # pre-grasp
        robot.arm.move(pose, max_speed=2, max_acc=1)
        pose[2] = cubeSize - grasp_depth
        robot.arm.move(pose)
        robot.hand.close()
        pose[2] = 0.1 # pre-grasp
        robot.arm.move(pose)
        robot.arm.move(home_pose, max_speed=1, max_acc=0.2)
        stack_pose[2] = (i + 1) * cubeSize - grasp_depth
        robot.arm.touch(stack_pose)
        robot.hand.move(128)
        robot.arm.move(home_pose, max_speed=2, max_acc=1)

    time.sleep(1)
    robot.disconnect()
