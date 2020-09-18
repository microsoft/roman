import os
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)

# this brings in most of the relevant parts. It requires numpy, scipy and pybullet
import math
import random
from roman import *

def rand(max):
    return (random.random()-0.5)*max

if __name__ == '__main__':
    # connect to robot.  
    robot = connect(use_sim=True)
    
    # move the arm in small relative increments
    t_step = 0.1
    r_step = 1
    for i in range(5):
        robot.move_simple(rand(t_step), rand(t_step), rand(t_step), rand(r_step), max_speed = 0.5)

    # now step instead (sustain the move for a fixed time)
    for i in range(40):
        robot.step(rand(t_step), rand(t_step), rand(t_step), rand(r_step), max_speed = 1, max_acc=1)

    robot.disconnect()
    
