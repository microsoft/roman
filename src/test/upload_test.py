import sys
import numpy as np
import math 
import time
import random
import os
parentdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, parentdir)
print(parentdir)

import robot

arm = robot.UR5Arm()
script = arm.generate_urscript()
# outF = open("complete.script", "w")
# outF.writelines(script)
# outF.close()
print(script)