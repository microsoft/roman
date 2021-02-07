################################################################################################################################
## pybullet implementation of the URScript functions needed by the control layer. 
################################################################################################################################
import math
import pybullet as pb
import numpy as np
from scipy.spatial.transform import Rotation
from ..ur.realtime.constants import *

class URArm:
    '''PyBullet-specific implementation of the simulated arm'''
    # sim-specific constants
    SIM_MAX_JOINT_FORCE = 100

    def __init__(self, body_id, base_joint_id, tcp_id, sim_time_step = 1/240.):
        self.body_id = body_id
        self.base_joint_id = base_joint_id
        self.ft_sensor_id = base_joint_id+7
        self.tcp_id = tcp_id
        self.joint_ids = range(base_joint_id, base_joint_id+6)
        self.sim_time_step = sim_time_step

    def reset(self):
        # start position is along the x axis, in negative direction 
        start_positions = [0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]
        for i in range(6):
            pb.resetJointState(self.body_id, self.base_joint_id + i, start_positions[i])            
            pb.setJointMotorControl2(self.body_id, 
                                    self.base_joint_id + i, 
                                    controlMode=pb.VELOCITY_CONTROL,
                                    targetVelocity = 0,
                                    force = URArm.SIM_MAX_JOINT_FORCE)

        pb.enableJointForceTorqueSensor(self.body_id, self.ft_sensor_id, True)
        self.ft_bias = np.zeros(6)
        pb.stepSimulation()
        self.ft_bias[:] = self.ur_get_tcp_sensor_force()
        #self._debug_dump()
    
    def get_inverse_kin(self, pose):
        '''Calculates the joint angles that corespond to the specified tool pose.'''
        rot = Rotation.from_rotvec(pose[3:6]).as_quat()
        joints = pb.calculateInverseKinematics(self.body_id, self.tcp_id, pose[0:3], rot)
        return joints[:6]

    def get_actual_tcp_pose(self):
        link_state = pb.getLinkState(self.body_id, self.tcp_id, computeLinkVelocity = 0, computeForwardKinematics = 1)
        pos = link_state[4]
        q = link_state[5]
        rot = Rotation.from_quat(q).as_rotvec()
        return [pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]]

    def get_actual_tcp_speed(self):
        link_state = pb.getLinkState(self.body_id, self.tcp_id, computeLinkVelocity = 1, computeForwardKinematics = 1)
        pos = link_state[6]
        rot = link_state[7]
        return [pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]]

    def get_actual_joint_positions(self):
        joint_states = pb.getJointStates(self.body_id, self.joint_ids)
        joint_positions = [state[0] for state in joint_states]
        return joint_positions

    def get_actual_joint_speeds(self):
        joint_states = pb.getJointStates(self.body_id, self.joint_ids)
        joint_velocities = [state[1] for state in joint_states]
        return joint_velocities

    def get_target_tcp_pose(self):
        return self.get_actual_tcp_pose()

    def get_target_tcp_speed(self):
        return self.get_actual_tcp_speed()

    def get_target_joint_positions(self):
        return self.get_actual_joint_positions()

    def get_target_joint_speeds(self):
        return self.get_actual_joint_speeds()

    def get_tcp_force(self):
        return self.ur_get_tcp_sensor_force()

    def ur_get_tcp_acceleration(self):
        return [0,0,0]

    def get_joint_torques(self):
        joint_states = pb.getJointStates(self.body_id, self.joint_ids)
        joint_torques = [state[3] for state in joint_states]
        return joint_torques


    def ur_get_tcp_sensor_force(self):
        ft_joint_state = pb.getJointState(self.body_id, self.ft_sensor_id)
        ft_force = np.array(ft_joint_state[2]) * -1 # getJointState returns a reaction force, we need the action
        # rotate to base coordinates
        ft_link_state = pb.getLinkState(self.body_id, self.ft_sensor_id)
        ft_orientation = ft_link_state[1]
        rot = Rotation.from_quat(ft_orientation)
        force = rot.apply(ft_force[:3])
        moments = rot.apply(ft_force[3:])
        return np.append(force, moments) - self.ft_bias

    def speedj(self, speed, max_acc):
        for i in range(6):
            new_speed = speed[i]
            pb.setJointMotorControl2(self.body_id, 
                                    self.base_joint_id + i, 
                                    controlMode=pb.VELOCITY_CONTROL,
                                    targetVelocity = new_speed,
                                    force = URArm.SIM_MAX_JOINT_FORCE)

    def set_payload(self, m, cog):
        pass

    def set_tcp(self, pose):
        pass

    def _debug_dump(self):
        print("joints:")

        for i in range(pb.getNumJoints(self.body_id)):
            ji = pb.getJointInfo(self.body_id, i)
            print(f"{i}: ix={ji[0]}, name={ji[1]}, type={ji[2]}, link={ji[12]}, link={ji[12]}, , llim={ji[8]}, ulim={ji[9]}, force={ji[10]}, vel={ji[11]} ")

        print("dynamics:")
        for i in range(pb.getNumJoints(self.body_id)):
            di = pb.getDynamicsInfo(self.body_id, i)
            print(di)