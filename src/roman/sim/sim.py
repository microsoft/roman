import numpy as np
import pybullet as p
import time
import os
p.connect(p.GUI)
urdfFile = os.path.join(os.path.dirname(__file__), 'roman.urdf')
p.loadURDF(urdfFile)

p.setGravity(0,0,-10)
p.setJointMotorControl2(0, 1, controlMode=p.VELOCITY_CONTROL, force=10)

while True:
    p.stepSimulation()
    time.sleep(1./240.)
