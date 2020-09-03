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

# joint-linear speed 
def ur_speed_joint_linear(target, max_speed, max_acc, max_final_speed):
    joint_positions = get_actual_joint_positions()
    joint_speeds = get_target_joint_speeds() 
    itime = 100000.0
    i = 0
    while i < 6:
        dist = target[i] - joint_positions[i]
        if (dist > UR_EPSILON) or (dist < -UR_EPSILON):
            sign = fabs(dist)/dist
            dist = fabs(dist)
            if joint_speeds[i]*sign < 0: # moving in the wrong direction
                dist = dist +  0.5*(joint_speeds[i]*joint_speeds[i])/max_acc # add the distance this joint will travel before it can reverse
            #ur:end
            
            if dist <= 0.5*(joint_speeds[i]*joint_speeds[i]-max_final_speed*max_final_speed)/max_acc: 
                sign = -sign # we are close to the goal and need to decelerate
            #ur:end
            speed = fabs(joint_speeds[i] + sign * max_acc * UR_TIME_SLICE) 
            if speed > max_speed:
                speed = max_speed
            #ur:end 
            # this is (the inverse of) how long it would take to move to target given the updated speed 
            tmp_itime = speed/dist
            if tmp_itime < itime:
                itime = tmp_itime
            
            #ur:end   
        #ur:end
        i = i + 1
    #ur:end   
 
    return [
        (target[0]-joint_positions[0])*itime,
        (target[1]-joint_positions[1])*itime,
        (target[2]-joint_positions[2])*itime,
        (target[3]-joint_positions[3])*itime,
        (target[4]-joint_positions[4])*itime,
        (target[5]-joint_positions[5])*itime
    ]
#ur:end

# joint-linear speed 
def ur_speed_tool_linear(target, max_speed, max_acc):
    target = ur_pose(target)
    tcp_pose = get_actual_tcp_pose()
    dist = fabs(point_dist(tcp_pose, target)) # TODO: this needs to be pose_dist instead
    if dist < UR_TOOL_POSITION_TOLERANCE:
        return [0,0,0,0,0,0]
    #ur:end
    desired_speed = sqrt(2*dist*max_acc) 
    if desired_speed > max_speed:
        desired_speed = max_speed
    #ur:end
    
    js = get_target_joint_speeds()
    speed = norm([js[0], js[1], js[2]]) 
    if speed > desired_speed:
        speed = desired_speed
    #ur:end

    # given the speed, this is how much we would like to move in a time slice
    delta = UR_TIME_SLICE * speed
    alpha = delta/dist
    if alpha > 1:
        alpha = 1
    #ur:end
    waypoint = interpolate_pose(tcp_pose, target, alpha)
    wj = get_inverse_kin(waypoint)
    js = ur_speed_joint_linear(wj, max_speed, max_acc, desired_speed)
    return js
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
def ur_get_target_speed(cmd_time, id, kind, target, max_speed, max_acc, force_low_bound, force_high_bound, contact_handling):
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
    elif kind == UR_CMD_KIND_MOVE_JOINTS_SPEED:
        cmd = target
        acc = max_acc
    elif kind == UR_CMD_KIND_MOVE_JOINTS_POSITION: # this covers UR_CMD_KIND_MOVE_TOOL_POSE too, see interface.py 
        cmd = ur_speed_joint_linear(target, max_speed, max_acc, 0)
        acc = max_acc
    elif kind == UR_CMD_KIND_MOVE_TOOL_LINEAR:
        cmd = ur_speed_tool_linear(target, max_speed, max_acc)
        acc = max_acc
    #ur:end
    global ctrl_is_moving
    ctrl_is_moving = (norm(cmd) > UR_SPEED_TOLERANCE) or (norm(get_actual_joint_speeds()) > UR_SPEED_TOLERANCE)

    # update state
    return [cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], acc]
#ur:end
