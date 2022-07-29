import pybullet as pb
from .constants import *
from .urlib import *
from .control import *

ur_last_state = [0, 0, 0, 0, 0, 0]

def ur_get_status():
    global ur_last_state
    return ur_last_state

def ur_drive(time, id, kind, target, max_speed, max_acceleration, force_low_bound, force_high_bound, controller, controller_args):
    res = ur_get_target_speed(time,
                                id,
                                kind,
                                target,
                                max_speed,
                                max_acceleration,
                                force_low_bound,
                                force_high_bound,
                                controller,
                                controller_args)
    global ur_last_state
    ur_last_state = res[7:]
    speedj(res[:6], res[6])



