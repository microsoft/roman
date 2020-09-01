from math import *
from .constants import *
from .urlib import *

################################################################################################################################
## control.py
## Implements the speed and position arm controllers. 
## Both controllers use the FT sensor to provide stop-on-contact behavior.
## Both controllers implement a deadman switch of 100 ms, thus requiring the client to call the drive fn at 10Hz or more 
## even if the command is the same.
################################################################################################################################

# generates a trapezoidal speed profile for one joint 
def ur_speed_from_joint_position(current_pos, current_speed, target_pos, target_speed, max_speed, acc):
    distance = target_pos - current_pos

    lim = 0.0001
    if (distance * current_speed) > 0: # is the joint rotating in the right direction?
        lim = lim + 0.5*(current_speed*current_speed-target_speed*target_speed)/acc
    #ur:end
    # move with maximum speed as long as we are far away from the goal, and decelerate when we get close
    if (lim < 0) or (distance*distance > lim*lim):
        if distance<0: 
            return -max_speed
        #ur:end
        return max_speed
    #ur:end
    return 0.0
#ur:end

# generates a trapezoidal speed profile for each joint, independent of each other 
def ur_speed_from_joint_positions(target_position, target_speed, max_speed, max_acc):
    joint_positions = get_actual_joint_positions()
    joint_speeds = get_actual_joint_speeds() 
    return [
        ur_speed_from_joint_position(joint_positions[0], joint_speeds[0], target_position[0], target_speed[0], max_speed, max_acc),
        ur_speed_from_joint_position(joint_positions[1], joint_speeds[1], target_position[1], target_speed[1], max_speed, max_acc),
        ur_speed_from_joint_position(joint_positions[2], joint_speeds[2], target_position[2], target_speed[2], max_speed, max_acc),
        ur_speed_from_joint_position(joint_positions[3], joint_speeds[3], target_position[3], target_speed[3], max_speed, max_acc),
        ur_speed_from_joint_position(joint_positions[4], joint_speeds[4], target_position[4], target_speed[4], max_speed, max_acc),
        ur_speed_from_joint_position(joint_positions[5], joint_speeds[5], target_position[5], target_speed[5], max_speed, max_acc)
    ]
#ur:end

# joint-linear speed 
def ur_speed_linear_from_joint_positions(target_position, target_speed, max_speed, max_acc):
    joint_positions = get_actual_joint_positions()
    joint_speeds = get_target_joint_speeds() 
    itime = 1.0
    i = 0
    while i < 6:
        dist = target_position[i] - joint_positions[i]
        if (dist > 0.0001) or (dist < -0.0001):
            sign = fabs(dist)/dist
            dist = sign * dist
            if joint_speeds[i]*sign < 0: # moving in the wrong direction
                dist = dist +  0.5*(joint_speeds[i]*joint_speeds[i])/max_acc # add the distance this joint will travel before it can reverse
            #ur:end
            
            if dist <= 0.5*(joint_speeds[i]*joint_speeds[i]-target_speed[i]*target_speed[i])/max_acc: 
                sign = -sign # we are close to the goal and need to decelerate
            #ur:end
            speed = fabs(joint_speeds[i] + sign * max_acc * 0.008) 
            if speed > max_speed:
                speed = max_speed
            #ur:end 

            # this is how much the joint can move in this time slice
            tmp_itime = speed/dist
            if tmp_itime < itime:
                itime = tmp_itime
            #ur:end   
        #ur:end
        i = i + 1
    #ur:end   
 
    return [
        (target_position[0]-joint_positions[0])*itime,
        (target_position[1]-joint_positions[1])*itime,
        (target_position[2]-joint_positions[2])*itime,
        (target_position[3]-joint_positions[3])*itime,
        (target_position[4]-joint_positions[4])*itime,
        (target_position[5]-joint_positions[5])*itime
    ]
#ur:end

# state globals
ctrl_last_cmd_id = 0
ctrl_last_loop_time = 0
ctrl_is_contact = False
ctrl_is_moving = False
ctrl_is_deadman = False

def ur_get_status():
    global ctrl_last_loop_time
    global ctrl_last_cmd_id
    global ctrl_is_contact
    global ctrl_is_moving
    global ctrl_is_deadman
    if ctrl_is_moving:
        is_moving = 1
    else:
        is_moving = 0
    #ur:end
    if ctrl_is_contact:
        is_contact = 1
    else:
        is_contact = 0
    #ur:end
    if ctrl_is_deadman:
        is_deadman = 1
    else:
        is_deadman = 0
    #ur:end
    return [ctrl_last_loop_time, ctrl_last_cmd_id, is_moving, is_contact, is_deadman]
#ur:end


# verifies that the control loop runs fast enough. 
def ur_check_loop_delay(loop_time):
    time = ur_get_time()
    delay = time - loop_time
    if delay >= 0.016:
        textmsg("drive loop delayed: ", delay)
    #ur:end
    return time
#ur:end

# Generate a target speed based on the latest command and current state
def ur_get_target_speed(cmd_time, id, kind, target_speed, max_acc, force_low_bound, force_high_bound, contact_handling, target_position, max_speed):
    # verify we are not running behind
    global ctrl_last_loop_time
    ctrl_last_loop_time = ur_check_loop_delay(ctrl_last_loop_time) 
    # determine external forces and motion status
    force_limit_reached = ur_force_limit_exceeded(force_low_bound, force_high_bound) 
    # only flip is_contact back to false when we receive a new command 
    global ctrl_last_cmd_id
    global ctrl_is_contact
    ctrl_is_contact = force_limit_reached #or (ctrl_is_contact and (id == ctrl_last_cmd_id)) 
    ctrl_last_cmd_id = id
    global ctrl_is_deadman
    was_deadman = ctrl_is_deadman
    ctrl_is_deadman = (ctrl_last_loop_time - cmd_time) > UR_DEADMAN_SWITCH_LIMIT

    cmd = UR_ZERO
    acc = UR_FAST_STOP_ACCELERATION
    # determine desired speed 
    if ctrl_is_deadman:
        if not was_deadman:
            textmsg("deadman")
        #ur:end
    elif ctrl_is_contact: 
        textmsg("force limit STOP")
    elif kind == UR_CMD_KIND_MOVE_JOINTS_POSITION:
        #cmd = ur_speed_from_joint_positions(target_position, target_speed, max_speed, max_acc)
        cmd = ur_speed_linear_from_joint_positions(target_position, target_speed, max_speed, max_acc)
        acc = max_acc
    elif kind == UR_CMD_KIND_MOVE_JOINTS_SPEED:
        cmd = target_speed
        acc = max_acc
    #ur:end
    global ctrl_is_moving
    ctrl_is_moving = (norm(cmd) > UR_SPEED_TOLERANCE) or (norm(get_actual_joint_speeds()) > UR_SPEED_TOLERANCE)

    # update state
    return [cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], acc]
#ur:end
