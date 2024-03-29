import os
import time
import math
import pybullet as pb
import pybullet_data
import functools
import numpy as np

class Robotiq3FGripper:
    _PINCH_LIMIT = 175 # in pinch mode the fingers can only travel part way (until they touch).

    '''PyBullet-specific implementation of the simulated gripper'''
    # names must match URDF.
    jointNames = [
        # finger A
        'finger_middle_joint_1', 'finger_middle_joint_2', 'finger_middle_joint_3',
        # finger B
        'finger_2_joint_1', 'finger_2_joint_2', 'finger_2_joint_3',
        # finger C
        'finger_1_joint_1', 'finger_1_joint_2', 'finger_1_joint_3',
        # scissor joints
        'palm_finger_2_joint', 'palm_finger_1_joint'
    ]

    fingerIndices = [slice(0, 3), slice(3, 6), slice(6, 9)]
    fingerTipIdx = [2, 5, 8]
    fingerAll = slice(0, 9)
    scissors = slice(9,11)

    def __init__(self, body_id):
        self.body_id = body_id
        self.joints = {}
        self.fingerTips = []
        numJoints = pb.getNumJoints(body_id)
        names = self.jointNames
        for i in range(numJoints):
            info = pb.getJointInfo(body_id, i)
            jointName = info[1].decode("utf-8")
            if jointName not in names:
                continue

            self.joints[jointName] = info
            jointID = info[0]

            if jointName in [names[ft] for ft in self.fingerTipIdx]:
                # default value is 0.5
                pb.changeDynamics(body_id, jointID, lateralFriction=10)
                self.fingerTips += [jointID]
                pb.enableJointForceTorqueSensor(self.body_id, jointID, True)

            pb.resetJointState(self.body_id, jointID, 0)
            pb.setJointMotorControl2(body_id, jointID, pb.VELOCITY_CONTROL, targetVelocity=0, force=100)

        self.jointIDs = [self.joints[name][0] for name in Robotiq3FGripper.jointNames]
        self._targets = [0, 0, 0]
        self._current_target = [0, 0, 0]

        joint1Stops = np.array(range(256)) / 255.
        joint2Stops = np.zeros(256)
        joint2Stops[128:] = np.array(range(128)) / 128.
        joint3Stops = np.zeros(256)
        joint3Stops[:192] = np.array(range(0, -192, -1)) / 192.
        joint3Stops[192:] = np.array(range(-192, -128)) / 192.
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

        # grasp modes: normal, pinch, wide, scissors, narrow
        # urdf joint limit are [0.16, -0.25], but 0.16 causes a self collision in pinch mode,
        # which messes up the FT sensor reading, so we use 0.13 instead
        self.modeStopsB = [0, 0.13, -0.25, -0.25, 0.07]
        self.modeStopsC = [0, -0.13, 0.25, 0.25, -0.07]
        self.last_joint_positions  = np.zeros(len(self.jointNames))
        self.last_joint_speeds  = np.zeros(len(self.jointNames))
        self._mode_limit = 255

    def reset(self):
        self.read()
        self.set_mode(0)
        self.move(0, 255, 255)

    def move(self, position, speed, force):
        for finger in range(3):
            self.move_finger(finger, position, speed, force)

    def move_finger(self, finger, position, speed, force):
        joints = self.jointIDs[self.fingerIndices[finger]]
        cnt = len(joints)
        self.read()
        is_new_cmd = self._targets[finger] != position
        contact = self._check_contact_threshold(finger, force)

        # compute the next intermediate position if not in contact,
        # else keep the last intermediate target
        if is_new_cmd or not contact:
            self._targets[finger] = position
            # This is a very rough approximation of how the real hand moves. Needs more work.
            speed = 2 + speed // 8 # min 2 to avoid stall
            current = self.positions()[0]
            inc = min(abs(position - current), speed)
            if position < current:
                inc = -inc
            position = current + inc
            self._current_target[finger] = min(position, self._mode_limit)  # account for pinch mode

        joint_positions = self.jointStops[self._current_target[finger], self.fingerIndices[finger]]
        forces = [force + 1] * cnt
        pb.setJointMotorControlArray(self.body_id, joints, pb.POSITION_CONTROL, targetPositions=joint_positions, forces=forces)

    def stop(self):
        self.read()
        cnt = len(self.jointIDs)
        pb.setJointMotorControlArray(self.body_id, self.jointIDs, pb.VELOCITY_CONTROL, targetVelocities=[0] * cnt, forces=[255] * cnt)
        self._targets[:] = self.positions()

    def _check_contact_threshold(self, finger, force):
        ft_force = self.get_finger_sensor_force(finger)
        return np.any(np.abs(ft_force) > 1 + abs(force) / 64)

    def set_mode(self, mode):
        self.read()
        self._mode = mode
        ix = int(mode / 2)
        joints = [self.jointIDs[9], self.jointIDs[10]]
        positions = [self.modeStopsB[ix], self.modeStopsC[ix]]
        pb.setJointMotorControlArray(self.body_id, joints, pb.POSITION_CONTROL, targetPositions=positions, forces=[100, 100])

        # in pinch mode the fingers can only travel part way (until they touch).
        self._mode_limit = self._PINCH_LIMIT if mode == 2 else 255

    def read(self):
        newStates = pb.getJointStates(self.body_id, self.jointIDs)
        jointPos = np.array([s[0] for s in newStates])
        self._is_moving = not np.allclose(jointPos, self.last_joint_positions, atol=0.0001)
        self._object_detected = not self._is_moving and not np.allclose(self._targets, self.positions(), atol=1)
        self.last_joint_speeds[:] = [s[1] for s in newStates]
        self.last_joint_positions = jointPos

    def mode(self):
        return self._mode

    def positions(self):
        # hackish way of computing the finger position, by considering only the base joint of each finger.
        # Needs to be in sync with the implementaiton of "move" and the position tables
        pos = np.int32(self.last_joint_positions[:9:3] * 256)
        pos = [255 if p >= self._mode_limit - 1 else p for p in pos]
        return pos

    def targets(self):
        return self._targets

    def object_detected(self):
        return self._object_detected

    def is_moving(self):
        return self._is_moving

    def get_finger_sensor_force(self, finger):
        ftid = self.fingerTips[finger]
        ft_joint_state = pb.getJointState(self.body_id, ftid)
        ft_force = np.array(ft_joint_state[2]) * -1 # getJointState returns a reaction force, we need the action
        return ft_force

