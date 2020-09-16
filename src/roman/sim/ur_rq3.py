import numpy as np
import pybullet as pb
import os
import math
from . import ur

################################################################
## configures the simulated robot and environment
################################################################
class SimEnv(object):
    # from urdf
    UR_BASE_JOINT_ID = 1 
    #UR_TCP_ID = 22 # use this for TCP set at [0,0,0]
    UR_TCP_ID = 23 # use this for TCP set at [0,0,0.260] 
    TIME_STEP = 1/240.

    '''
    Loads the default environment (arm/hand/table urdf files) 
    and allows further configuration of the cameras, objects present in the scene etc.
    '''
    def __init__(self, urdf='ur_rq3.urdf', useGUI = True):
        self._arm_urdf = os.path.join(os.path.dirname(__file__), urdf)
        self._useGUI = useGUI
        self.__time = 0.0

    def connect(self):
        if self._useGUI:
            pb.connect(pb.GUI)
            pb.resetDebugVisualizerCamera(1.5, -30, -15, cameraTargetPosition=[-0.4, 0, 0.3])
        else:
            pb.connect(pb.DIRECT)
        self.reset()

    def reset(self):
        pb.resetSimulation()
        pb.setGravity(0,0,-10)
        arm_id = pb.loadURDF(self._arm_urdf, baseOrientation = pb.getQuaternionFromEuler([0, 0, math.pi]))
        self.arm = ur.URArm(arm_id, SimEnv.UR_BASE_JOINT_ID, SimEnv.UR_TCP_ID, SimEnv.TIME_STEP)
        self.arm.reset()

    def update(self):
        pb.stepSimulation()
        self.__time += SimEnv.TIME_STEP
        
    def time(self):
        return self.__time
   
    def disconnect(self):
        pb.disconnect()
 