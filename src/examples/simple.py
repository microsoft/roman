# this deals with the case when RoMan is not installed as a package
import os
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)

# this brings in most of the relevant parts. It requires numpy, scipy and pybullet
from roman import *
import math

if __name__ == '__main__':
    # connect to robot. Use connect_real() to start up the real robot. 
    robot = connect(use_sim=True)

    # arm poses can be expresses in tool space (carthesian coordinates xyzrpy) ...
    p1 = arm.Tool(-0.4, -0.4, 0.2, 0, math.pi, 0)

    # ... or joint angles
    p2 = arm.Joints(0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0)

    # default way to move blocks until the arm reaches the specified pose
    for i in range(3):
        robot.arm.move(p1, max_speed=1, max_acc=0.5)
        robot.arm.move(p2, max_speed=2, max_acc=2)
    
    robot.disconnect()