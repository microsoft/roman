################################################################################################################################
## Redirects the UR functions needed by the control script to the simulator. 
################################################################################################################################
import pybullet as pb
import numpy as np
from scipy.spatial.transform import Rotation
import math
from .constants import *

# this abstracts the simulator
sim = None

#*****************************************************************************
# URScript-like functions needed by our script layer.
#*****************************************************************************
def ur_get_time():
    return sim.time()

def get_inverse_kin(pose):
    return sim.arm.get_inverse_kin(pose)

def get_actual_tcp_pose():
    return sim.arm.get_actual_tcp_pose()

def get_actual_tcp_speed():
    return sim.arm.get_actual_tcp_speed()

def get_actual_joint_positions():
    return sim.arm.get_actual_joint_positions()

def get_actual_joint_speeds():
    return sim.arm.get_actual_joint_speeds()

def get_target_tcp_pose():
    return sim.arm.get_target_tcp_pose()

def get_target_tcp_speed():
    return sim.arm.get_target_tcp_speed()

def get_target_joint_positions():
    return sim.arm.get_target_joint_positions()

def get_target_joint_speeds():
    return sim.arm.get_target_joint_speeds()

def get_tcp_force():
    return sim.arm.get_tcp_force()

def get_joint_torques():
    return sim.arm.get_joint_torques()

def ur_get_tcp_sensor_force(__unused = 0):
    return sim.arm.ur_get_tcp_sensor_force()

def ur_get_tcp_acceleration():
    return sim.arm.ur_get_tcp_acceleration()

def speedj(speed, max_acc):
    sim.arm.speedj(speed, max_acc)

def set_payload(m, cog):
    sim.arm.set_payload(m, cog)

def set_tcp(pose):
    sim.arm.set_tcp(pose)

#******************************************************************************
# various other UR script and custom functions that are not simulation-specific
#******************************************************************************

def textmsg(s1, s2=""):
    print(str(s1)+str(s2))

def norm(v):
    ''' Norm function, as defined by urscript'''
    return np.linalg.norm(v)

def point_dist(p_from, p_to):
    '''
    Point distance, as defined by urscript.
    Returns the distance between the two tool positions (without considering rotations)
    '''
    return math.dist(p_from[:3], p_to[:3])

def pose_sub(p_to, p_from):
    ''' Pose subtraction, as defined by urscript.'''
    t = np.subtract(p_to[:3], p_from[:3])
    r_to = Rotation.from_rotvec(p_to[3:])
    r_from = Rotation.from_rotvec(p_from[3:])
    r = (r_from.inv() * r_to).as_rotvec()
    return np.concatenate((t, r))

def pose_add(p1, p2):
    ''' Pose addition, as defined by urscript.'''
    t = np.add(p1[:3], p2[:3])
    r1 = Rotation.from_rotvec(p1[3:])
    r2 = Rotation.from_rotvec(p2[3:])
    r = (r1*r2).as_rotvec()
    return np.concatenate((t, r))

def interpolate_pose(p_from, p_to, alpha):
    ''' Linear interpolation of tool position and orientation, as defined by urscript.'''
    delta = pose_sub(p_to, p_from)
    return pose_add(p_from, delta*alpha)

def sqrt(a):
    return math.sqrt(a)

def ur_pose(v):
    return v

def ur_check_loop_delay(last_loop_time):
    return ur_get_time()

def ur_force_limit_exceeded(low_bound, high_bound):
    ft = ur_get_tcp_sensor_force()
    return np.any(np.greater(low_bound, ft)) or np.any(np.greater(ft, high_bound))

