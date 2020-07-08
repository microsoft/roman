################################################################################################################################
## Contains the sim (pybullet) equivalent of the UR functions needed by the control script. 
################################################################################################################################
import pybullet as pb
import numpy as np
from robot.URScripts.constants import *

# sim-specific constants
SIM_MAX_JOINT_FORCE = 1000
SIM_BODY_ID = 0
SIM_BASE_JOINT_ID = 1
SIM_FT_SENSOR_ID = 8
SIM_TCP_ID = 7
SIM_JOINT_IDS = range(SIM_BASE_JOINT_ID, SIM_BASE_JOINT_ID+6)
SIM_PERIOD = 1/240.

def norm(vec6):
    ''' Norm function as defined by urscript'''
    return np.linalg.norm(vec6)

def get_inverse_kinematics(pose):
    '''Calculates the joint angles that corespond to the specified tool pose.'''
    joints = pb.calculateInverseKinematics(SIM_BODY_ID, SIM_TCP_ID, pose[0:3], pb.getQuaternionFromEuler(pose[3:6]))
    return joints[:6]

def get_actual_tcp_pose():
    link_state = pb.getLinkState(SIM_BODY_ID, SIM_TCP_ID, computeLinkVelocity = 0, computeForwardKinematics = 1)
    pos = link_state[0]
    rot = pb.getEulerFromQuaternion(link_state[1])
    return [pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]]

def get_actual_tcp_speed():
    link_state = pb.getLinkState(SIM_BODY_ID, SIM_TCP_ID, computeLinkVelocity = 1, computeForwardKinematics = 1)
    pos = link_state[6]
    rot = link_state[7]
    return [pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]]

def get_actual_joint_positions():
    joint_states = pb.getJointStates(SIM_BODY_ID, SIM_JOINT_IDS)
    joint_positions = [state[0] for state in joint_states]
    return joint_positions

def get_actual_joint_speeds():
    joint_states = pb.getJointStates(SIM_BODY_ID, SIM_JOINT_IDS)
    joint_velocities = [state[1] for state in joint_states]
    return joint_velocities

def get_target_tcp_pose():
    return [0,0,0,0,0,0]

def get_target_tcp_speed():
    return [0,0,0,0,0,0]

def get_target_joint_positions():
    return [0,0,0,0,0,0]

def get_target_joint_speeds():
    return [0,0,0,0,0,0]

def get_tcp_force():
    return [0,0,0,0,0,0]

def get_joint_torques():
    joint_states = pb.getJointStates(SIM_BODY_ID, SIM_JOINT_IDS)
    joint_torques = [state[3] for state in joint_states]
    return joint_torques


def ur_get_tcp_sensor_force(critical_section_already_acquired = False):
    tcp_state = pb.getJointState(SIM_BODY_ID, SIM_TCP_ID)
    return tcp_state[2]

def speedj(speed, max_acc):
    current = get_actual_joint_speeds()
    for i in range(6):
        if current[i] < speed[i]:
            new_speed = min(speed[i], current[i] + max_acc*SIM_PERIOD)    
        elif current[i] > speed[i]:
            new_speed = max(speed[i], current[i] - max_acc*SIM_PERIOD)  
        else:
            new_speed = 0
        pb.setJointMotorControl2(SIM_BODY_ID, 
                                SIM_BASE_JOINT_ID + i, 
                                controlMode=pb.VELOCITY_CONTROL,
                                targetVelocity = new_speed,
                                force = SIM_MAX_JOINT_FORCE)

def set_payload(m, cog):
    pass

def set_tcp(pose):
    pass

def textmsg(s1, s2):
    print(s1+s2)

def ur_pose(v):
    return v

def ur_get_time():
    return 0

def ur_check_loop_delay(last_loop_time):
    return ur_get_time()

def ur_force_limit_exceeded(low_bound, high_bound):
    ft = ur_get_tcp_sensor_force()
    return np.any(np.greater(low_bound, ft)) or np.any(np.greater(ft, high_bound))

def ur_get_tcp_acceleration():
    return [0,0,0]

