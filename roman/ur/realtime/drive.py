import pybullet as pb
from .constants import *
from .urlib import *
from .control import *

def ur_drive(time, id, kind, target, max_speed, max_acceleration, force_low_bound, force_high_bound, contact_handling):
    target = ur_get_target_speed(time, 
                                id, 
                                kind, 
                                target, 
                                max_speed, 
                                max_acceleration, 
                                force_low_bound, 
                                force_high_bound, 
                                contact_handling)
    speedj(target[:6], target[6])
    


