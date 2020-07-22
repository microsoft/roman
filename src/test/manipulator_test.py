import sys
import numpy as np
import math 
import time
import random
import os
import time
from multiprocessing import Process, Pipe
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)
from roman import *

def arm_move():
    print(f"Running {__file__}::{arm_move.__name__}()")
    
    m = connect()
    cmd = arm.Command(target_position=arm.Tool(-0.4, -0.4, 0.3,0, math.pi/2, math.pi))
    while not m.arm_state.is_done():
        m.execute(cmd)

    m.disconnect()
    
def run():
    arm_move()
#env_test()
if __name__ == '__main__':
    run()
