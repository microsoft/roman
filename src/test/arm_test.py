import sys
import numpy as np
import math 
import time
import random
import os
import socket
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)
print(rootdir)
import robot


def basic_com_test():
    arm = robot.UR5Arm()
    arm.connect()

    # just attempt to move to the current location (should be a no-op)
    time.sleep(1)
    arm.read()
    p = arm.tool_pose()
    arm.movep(p)
    time.sleep(1)
    arm.disconnect()

basic_com_test()