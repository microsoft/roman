import numpy as np
import pybullet as p
import time
p.connect(p.GUI)
p.loadURDF("c:\\code\\manipulation\\sim\\roman.urdf")

p.setGravity(0,0,-10)
p.setJointMotorControl2(0, 1, controlMode=p.VELOCITY_CONTROL, force=10)

while True:
    p.stepSimulation()
    time.sleep(1./240.)
