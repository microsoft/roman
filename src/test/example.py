import sys
import numpy as np
import math 
import time
import random
import os

# set paths (if not installed as a package)
os.chdir(r"c:\\projects\\roman")
sys.path.append("c:\\projects\\roman")

import robot

def main():
    man = robot.connect()
    try:
        # close the hand 
        man.hand.close()

        # calibrate FT sensor
        man.sleep(1)

        # rotate tool a bit
        joints = np.array(man.arm.joint_positions())
        joints[5] = joints[5] + 0.25 
        man.arm.movej(joints)

        # print the tool pose
        print(man.arm.tool_pose)

    except KeyboardInterrupt:
        pass
    man.disconnect()

main()
