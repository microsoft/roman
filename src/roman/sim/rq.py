import os
import time
import math
import pybullet as pb
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict
import functools
import numpy as np

class Robotiq3FGripper:
    '''PyBullet-specific implementation of the simulated gripper'''
    # names must match URDF.
    jointNames = [
                    # finger A
                    'finger_middle_joint_1','finger_middle_joint_2','finger_middle_joint_3',
                    # finger B
                    'finger_2_joint_1','finger_2_joint_2','finger_2_joint_3',
                    # finger C
                    'finger_1_joint_1','finger_1_joint_2','finger_1_joint_3',
                    # scissor joints 
                    'palm_finger_2_joint', 'palm_finger_1_joint'
                ]

    fingerIndices = [slice(0, 3), slice(3, 6), slice(6, 9)]
    fingerAll = slice(0, 9)
    scissors = slice(9,11)

    def __init__(self, body_id):
        self.body_id = body_id
        self.joints = {}
        numJoints = pb.getNumJoints(body_id)
        names = self.jointNames
        for i in range(numJoints):
            info = pb.getJointInfo(body_id, i)
            jointName = info[1].decode("utf-8")
            if jointName not in names:
                continue 
            
            self.joints[jointName] = info
            jointID = info[0]
            pb.resetJointState(self.body_id, jointID, 0)    
            pb.setJointMotorControl2(body_id, jointID, pb.VELOCITY_CONTROL, targetVelocity=0, force=100)

        self.jointIDs = [self.joints[name][0] for name in Robotiq3FGripper.jointNames]
        self._targets = [0,0,0]

        joint1Stops = np.array(range(256))/255.
        joint2Stops = np.zeros(256)
        joint2Stops[128:] = np.array(range(128))/128.
        joint3Stops = np.zeros(256)
        joint3Stops[:192] = np.array(range(0,-192,-1))/192.
        joint3Stops[192:] = np.array(range(-192, -128))/192.
        # joint3Stops[:160] = range(0, -160, -1)
        # joint3Stops[160:] = range(-160, 64)
        # joint3Stops = joint3Stops / 256.
        self.jointStops = np.zeros((256, 9))
        self.jointStops[:, 0] = joint1Stops
        self.jointStops[:, 3] = joint1Stops
        self.jointStops[:, 6] = joint1Stops
        
        self.jointStops[:, 1] = joint2Stops
        self.jointStops[:, 4] = joint2Stops
        self.jointStops[:, 7] = joint2Stops

        self.jointStops[:, 2] = joint3Stops
        self.jointStops[:, 5] = joint3Stops
        self.jointStops[:, 8] = joint3Stops
        
   
    def reset(self):
        self.read()
        self.set_mode(0)
        self.move(0,255,255) 

    def move(self, position, speed, force):
        self.read()
        # This is a very rough approximation of how the real hand moves. Needs more work.
        joints = self.jointIDs[self.fingerAll]
        cnt = len(joints)
        positions = self.jointStops[position]
        forces = [force+1] * cnt
        pb.setJointMotorControlArray(self.body_id, joints, pb.POSITION_CONTROL, targetPositions=positions, forces=forces) 
        self._targets[0] = self._targets[1] = self._targets[2] = position 

    def move_finger(self, finger, position, speed, force):
        self.read()
        # This is a very rough approximation of how the real hand moves. Needs more work.
        joints = self.jointIDs[self.fingerIndices[finger]]
        cnt = len(joints)
        positions = self.jointStops[position, self.fingerIndices[finger]]
        forces = [force+1] * cnt
        pb.setJointMotorControlArray(self.body_id, joints, pb.POSITION_CONTROL, targetPositions=positions, forces=forces) 
        self._targets[finger] = position

    def stop(self):
        self.read()
        cnt = len(self.jointIDs)
        pb.setJointMotorControlArray(self.body_id, self.jointIDs, pb.VELOCITY_CONTROL, targetVelocities=[0] * cnt, forces=[255] * cnt) 
        self._targets[:] = self.positions()
    
    def set_mode(self, mode):
        # todo
        self._mode = mode

    def read(self):
        self.jointStates = pb.getJointStates(self.body_id, self.jointIDs)

    def mode(self):
        return self._mode
    
    def positions(self):
        # hackish way of computing the finger position, by considering only the base joint of each finger. 
        # Needs to be in sync with the implementaiton of "move" and the position tables
        pos = [int(state[0]*256) for state in self.jointStates[:9:3]]
        return pos

    def targets(self):
        return self._targets

    def object_detected(self):
        return not self.is_moving() and not np.allclose(self._targets, self.positions(),1)

    def is_moving(self):
        return any((abs(state[1]) > 0.1 for state in self.jointStates)) # state[1] is velocity



