import numpy as np
import pybullet as p
import time
import os
import math

class UR5Arm(object):
    def __init__(self):
        urdfFile = os.path.join(os.path.dirname(__file__), 'roman.urdf')
        p.connect(p.DIRECT)

        self._uid = p.loadURDF(urdfFile)
        self._baseid = 1
        self._toolid = self._baseid + 5
        
        self._joints = range(self._baseid, self._baseid+6)
        self._links = range(self._baseid, self._baseid+6)

        p.resetJointState(self._uid, self._baseid + 1, -math.pi/2)
        p.resetJointState(self._uid, self._baseid + 2, math.pi/2)
        p.resetJointState(self._uid, self._baseid + 3, -math.pi/2)
        p.resetJointState(self._uid, self._baseid + 4, -math.pi/2)

    # position control
    def movep(self, tool_pose):
        joint_positions = p.calculateInverseKinematics(self._uid, endEffectorLinkIndex = self._toolid, targetPosition=tool_pose)
        print(joint_positions)
        p.setJointMotorControl2(self._uid, self._baseid, targetPosition=joint_positions[0], targetVelocity=0, controlMode=p.POSITION_CONTROL)
        p.setJointMotorControl2(self._uid, self._baseid+1, targetPosition=joint_positions[1], targetVelocity=0, controlMode=p.POSITION_CONTROL)
        p.setJointMotorControl2(self._uid, self._baseid+2, targetPosition=joint_positions[2], targetVelocity=0, controlMode=p.POSITION_CONTROL)
        p.setJointMotorControl2(self._uid, self._baseid+3, targetPosition=joint_positions[3], targetVelocity=0, controlMode=p.POSITION_CONTROL)
        p.setJointMotorControl2(self._uid, self._baseid+4, targetPosition=joint_positions[4], targetVelocity=0, controlMode=p.POSITION_CONTROL)
        p.setJointMotorControl2(self._uid, self._baseid+5, targetPosition=joint_positions[5], targetVelocity=0, controlMode=p.POSITION_CONTROL)

    def joint_positions(self):
        jointstates = p.getJointStates(self._uid, self._joints)       
        return [jointstates[0][0], jointstates[1][0], jointstates[2][0], jointstates[3][0], jointstates[4][0],jointstates[5][0]]

    def tool_pose(self):
        return p.getLinkState(self._uid,self._toolid, True, True)[0] 

    