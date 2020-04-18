import os
import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as p
import pybullet_data
import random
import sim_arm


class UR5Env(gym.Env):
  def __init__(self,
               urdfRoot=pybullet_data.getDataPath(),
               show_gui=False):
               
    self._urdfRoot = urdfRoot
    self._timeStep = 1. / 240.
    self._observation = []
    self._envStepCounter = 0
    self._renders = show_gui
    self._width = 224
    self._height = 224
    self.terminated = 0
    if self._renders:
      p.connect(p.GUI)
      p.resetDebugVisualizerCamera(1.3, 180, -41, [0.52, -0.2, -0.33])
    else:
      p.connect(p.DIRECT)
    
    self.seed()
    self.reset()
    observationDim = len(self.getExtendedObservation())
    
    action_dim = 3
    self._action_bound = 1
    action_high = np.array([self._action_bound] * action_dim)
    self.action_space = spaces.Box(-action_high, action_high, dtype=np.float32)
    self.observation_space = spaces.Box(low=0,
                                        high=255,
                                        shape=(self._height, self._width, 4),
                                        dtype=np.uint8)
    self.viewer = None

  def reset(self):
    self.terminated = 0
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    p.setTimeStep(self._timeStep)
    p.loadURDF(os.path.join(self._urdfRoot, "plane.urdf"), [0, 0, -1])

    p.loadURDF(os.path.join(self._urdfRoot, "table/table.urdf"), 0.5000000, 0.00000, -.820000,
               0.000000, 0.000000, 0.0, 1.0)

    xpos = 0.5 + 0.2 * random.random()
    ypos = 0 + 0.25 * random.random()
    ang = 3.1415925438 * random.random()
    orn = p.getQuaternionFromEuler([0, 0, ang])
    self.blockUid = p.loadURDF(os.path.join(self._urdfRoot, "block.urdf"), xpos, ypos, -0.1,
                               orn[0], orn[1], orn[2], orn[3])

    p.setGravity(0, 0, -10)
    self._arm = sim_arm.UR5Arm()
    p.stepSimulation()
    self._observation = self.getExtendedObservation()
    return np.array(self._observation)

  def __del__(self):
    p.disconnect()

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def getExtendedObservation(self):
    viewMat = [
        -0.5120397806167603, 0.7171027660369873, -0.47284144163131714, 0.0, -0.8589617609977722,
        -0.42747554183006287, 0.28186774253845215, 0.0, 0.0, 0.5504802465438843,
        0.8348482847213745, 0.0, 0.1925382763147354, -0.24935829639434814, -0.4401884973049164, 1.0
    ]
    projMatrix = [
        0.75, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0,
        -0.02000020071864128, 0.0
    ]

    img_arr = p.getCameraImage(width=self._width,
                               height=self._height,
                               viewMatrix=viewMat,
                               projectionMatrix=projMatrix)
    rgb = img_arr[2]
    np_img_arr = np.reshape(rgb, (self._height, self._width, 4))
    self._observation = np_img_arr
    return self._observation

  def step(self, action):
    # copy/paste from Kuka env
    dv = 0.01
    dx = action[0] * dv
    dy = action[1] * dv
    dz = -0.002
    da = action[2] * 0.1
    f = 0.3
    
    # for now, only the tool position is used
    current_tp = self._arm.tool_pose()
    target_tp = [current_tp[0] + dx,current_tp[1] + dy,current_tp[2] + dz]
    self._arm.movep(target_tp)
    p.stepSimulation()
    self._envStepCounter += 1

    self._observation = self.getExtendedObservation()
    done = self._envStepCounter > 1000
    reward = 0
    return np.array(self._observation), reward, done, {}
