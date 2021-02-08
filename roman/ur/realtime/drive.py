from . import control
from . import urlib

def ur_drive(time, id, kind, target, max_speed, max_acceleration, force_low_bound, force_high_bound, contact_handling):
    target = control.ur_get_target_speed(time, 
                                id, 
                                kind, 
                                target, 
                                max_speed, 
                                max_acceleration, 
                                force_low_bound, 
                                force_high_bound, 
                                contact_handling)
    urlib.speedj(target[:6], target[6])
    


