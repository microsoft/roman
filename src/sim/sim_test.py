import numpy as np
import pybullet as p
import time
import os
import sim_arm
import sim_env


def test_arm():
    p.connect(p.GUI)
    p.setGravity(0,0,-10)
    arm = sim_arm.UR5Arm()

    while True:
        arm.movep([0.35, 0.35, 0])
        p.stepSimulation()
        time.sleep(1./240.)


def test_env():
    env = sim_env.UR5Env(show_gui=True)
    while True:
        env.step([-1, 1, 1])

#test_arm()
test_env()