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
    # connect to robot. Use connect_real() to start up the real robot. 
    robot = connect_sim()
    
    # move the arm in small relative increments
    t_step = 0.1
    r_step = 0.1
    for i in range(10):
        robot.move_simple(rand(t_step), rand(t_step), rand(t_step), rand(r_step), max_speed = 1)
    
