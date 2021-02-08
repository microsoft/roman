import numpy as np
import os
import math
from . import ur
from . import rq
from . import simenv

################################################################
## configures the simulated robot and environment
################################################################
class SimEnv(simenv.SimEnv):
    # from urdf
    UR_BASE_JOINT_ID = 1 
    #UR_TCP_ID = 22 # use this for TCP set at [0,0,0]
    UR_TCP_ID = 23 # use this for TCP set at [0,0,0.260] 

    '''
    Creates the simulation (arm/hand/table).
    '''
    def __init__(self, urdf='ur_rq3.urdf', useGUI = True):
        super().__init__(useGUI)
        self.robot_urdf = os.path.join(os.path.dirname(__file__), urdf)

    def reset(self):
        super().reset()
        self.make_box([1,2,0.05], [-0.25,0,-0.025], color=(0.2,0.2,0.2,1))
        robot_id = self.loadURDF(self.robot_urdf, baseOrientation = [0, 0, math.pi])
        self.arm = ur.URArm(robot_id, SimEnv.UR_BASE_JOINT_ID, SimEnv.UR_TCP_ID, SimEnv.TIME_STEP)
        self.hand = rq.Robotiq3FGripper(robot_id)
        self.arm.reset()
        self.hand.reset()