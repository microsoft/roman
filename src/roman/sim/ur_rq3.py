import numpy as np
import pybullet as pb
import os
import math

################################################################
## configures the simulated robot and environment
################################################################
class SimEnv(object):
    '''
    Loads the default environment (arm/hand/table urdf files) 
    and allows further configuration of the cameras, objects present in the scene etc.
    '''
    def __init__(self, urdf='ur_rq3.urdf', useGUI = True):
        self.arm_urdf = os.path.join(os.path.dirname(__file__), urdf)
        self.useGUI = useGUI

    def connect(self):
        if self.useGUI:
            pb.connect(pb.GUI)
            pb.resetDebugVisualizerCamera(1.5, -30, -15, cameraTargetPosition=[-0.4, 0, 0.3])
        else:
            pb.connect(pb.DIRECT)

    def reset(self):
        pb.resetSimulation()
        self.arm_id = pb.loadURDF(self.arm_urdf, baseOrientation = pb.getQuaternionFromEuler([0, 0, math.pi]))

        # dump joints:
        # for i in range(pb.getNumJoints(self.arm_id)):
        #     ji = pb.getJointInfo(self.arm_id, i)
        #     print(f"{i}: ix={ji[0]}, name={ji[1]}, link={ji[12]}")

        #self.arm_id = pb.loadURDF(self.arm_urdf)
        base_joint = 1 # from urdf
        # start position is along the x axis, in negative direction 
        start_positions = [0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]
        for i in range(6):
            pb.resetJointState(self.arm_id, base_joint + i, start_positions[i])
        

    def update(self):
        pb.stepSimulation()
   
    def disconnect(self):
        pb.disconnect()

 