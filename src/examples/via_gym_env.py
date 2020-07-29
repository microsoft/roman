# this deals with the case when RoMan is not installed as a package
import os
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)

# this brings in most of the relevant parts. It requires numpy, scipy and pybullet
from roman import *
from roman import roman_env
import math

if __name__ == '__main__':
    # connect to robot. Use connect_real() to start up the real robot. 

    r_env = roman_env.RoManEnv(real_robot=False)
    #print('CURRENT STATE: ', obs['state'])
    #obs, rew, done, dbg = r_env.step([0, 0, 0, 0, 0])
    #print('CURRENT STATE: ', obs['state'])
    
    obs, rew, done, dbg = r_env.step([0.5, 0, 0, 0, 0])
    print(obs['state'])
    obs, rew, done, dbg = r_env.step([0, 0.5, 0, 0, 0])
    print(obs['state'])
    obs, rew, done, dbg = r_env.step([0, 0, 0.5, 0, 0])
    print(obs['state'])
    """
    obs, rew, done, dbg = r_env.step([0, 0, 0, math.pi , 0])
    print(obs['state'])
    obs, rew, done, dbg = r_env.step([0, 0, 0, 0, 1])
    print(obs['state'])
    obs = r_env.reset()
    print(obs['state'])
    obs, rew, done, dbg = r_env.step([0.5, 0, 0, 0, 0])
    print(obs['state'])
    r_env.close()
    """

    # arm poses can be expresses in tool space (carthesian coordinates xyzrpy) ...
    #p1 = arm.Tool(-0.4, -0.4, 0.2, 0, math.pi, 0)

    # ... or joint angles
    #p2 = arm.Joints(0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0)

    # default way to move blocks until the arm reaches the specified pose
    #for i in range(3):
    #    robot.arm.move(p1)
    #    robot.arm.move(p2, max_speed=2, max_acc=2)
    
