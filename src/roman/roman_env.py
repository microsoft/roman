import gym
from gym import spaces
import numpy as np
import math
from . import robot
#import robot

class RoManEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    init_state = [0, 0, 0, 0, 0]

    def __init__(self, real_robot=False):
        super().__init__()
        # What are the state dimensions?????? why are there 6 of them???
        self.action_space = spaces.Box(low=np.array([-2, -2, -2, -math.pi, 0]), high=np.array([2, 2, 2, math.pi, 1]), dtype=np.float32)
        # Video frames follow the channels, width, height convention.
        # How do we get the video frames, at least from the simulator???
        self.observation_space = spaces.Dict({'cam': spaces.Box(low=0, high=255, shape=(3, 224, 224), dtype=np.uint8), \
                                            'state': spaces.Box(low=np.array([-2, -2, -2, -math.pi, 0]), high=np.array([2, 2, 2, math.pi, 1]), dtype=np.float32)})
        if real_robot:
            self.robot = robot.connect_real()
        else:
            self.robot = robot.connect_sim()

    def step(self, action):
        assert self.action_space.contains(action)
        dx, dy, dz, dyaw, gripper_state = action
        self.robot.move_simple(dx, dy, dz, dyaw, gripper_state)
        obs = {'cam': None, 'state': self.robot.state}
        # TO DO: set the reward and the done flag in a problem instance-specific way.
        reward = 0
        done = False
        debug_info = {}
        return obs, reward, done, debug_info
    
    def reset(self):
        #self.robot.arm.move(self.init_state)
        # How does the robot class interface with the simulation? Is there such a thing as resetting the real robot? 
        # HOW DO WE DO RESETS???
        self.robot.reset()
        obs = {'cam': None, 'state': self.robot.state}
        return obs

    def close(self):
        self.robot.disconnect()

    #assert hand.state.position() == Position.CLOSED
    #hand.open(blocking=False)
    #time.sleep(2)
    #hand.read()
    #assert hand.state.position() == Position.OPENED
