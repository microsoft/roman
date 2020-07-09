from robot.URScripts.constants import *
from robot.URScripts.urlib import *

################################################################################################################################
## control.py
## Implements the speed and position arm controllers. 
## Both controllers use the FT sensor to provide stop-on-contact behavior.
## Both controllers implement a deadman switch of 100 ms, thus requiring the client to call the drive fn at 10Hz or more 
## even if the command is the same.
################################################################################################################################

# Computes the speed required to comply with the external force
def ur_force_control():
    #textmsg("Contact detected: ", external_force)
    return [0,0,0,0,0,0]
#ur:end

# generates a trapezoidal speed profile for one joint 
def ur_speed_from_joint_position(current_pos, current_speed, target_pos, target_speed, max_speed, acc):
    distance = target_pos - current_pos
    lim = 0
    if (distance * current_speed) > 0: # is the joint rotating in the right direction?
        lim = 0.5*current_speed*current_speed/acc
    #ur:end
    # move with maximum speed as long as we are far away from the goal, and decelerate when we get close
    if (distance*distance)>(lim*lim + 0.001 * (current_speed*current_speed / (max_speed*max_speed))):
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
    end
    if ctrl_is_contact:
        is_contact = 1
    else:
        is_contact = 0
    end
    if ctrl_is_deadman:
        is_deadman = 1
    else:
        is_deadman = 0
    end
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
    force_limit_reached = False #ur_force_limit_exceeded(force_low_bound, force_high_bound) 
    # only flip is_contact back to false when we receive a new command 
    global ctrl_last_cmd_id
    global ctrl_is_contact
    ctrl_is_contact = force_limit_reached or (ctrl_is_contact and (id == ctrl_last_cmd_id)) 
    ctrl_last_cmd_id = id
    global ctrl_is_deadman
    ctrl_is_deadman = False

    # determine desired speed 
    if force_limit_reached: 
        cmd = ur_force_control()
        max_acc = UR_FAST_STOP_ACCELERATION
        textmsg("force limit reached")
    elif (ctrl_last_loop_time - cmd_time) > UR_DEADMAN_SWITCH_LIMIT:
        cmd = UR_ZERO
        max_acc = UR_FAST_STOP_ACCELERATION
        ctrl_is_deadman = True
        textmsg("deadman")
    elif kind == UR_CMD_KIND_MOVE_JOINTS_POSITION:
        cmd = ur_speed_from_joint_positions(target_position, target_speed, max_speed, max_acc)
    else:
        cmd = target_speed
    #ur:end
    global ctrl_is_moving
    ctrl_is_moving = (norm(cmd) > UR_SPEED_TOLERANCE) or (norm(get_actual_joint_speeds()) > UR_SPEED_TOLERANCE)

    # update state
    return cmd
#ur:end
