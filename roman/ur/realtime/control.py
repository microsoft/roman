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
def get_joint_distance(current, target):
    delta = target - current
    if fabs(delta) > pi:
        alt = delta - 4 * pi
        i = 0
        while i < 5:
            if fabs(alt) < fabs(delta):
                delta = alt
            #ur:end
            alt = alt + 2 * pi
            i = i + 1
        #ur:end
    elif fabs(delta)<UR_JOINTS_POSITION_TOLERANCE:
        delta=0
    #ur:end
    return delta
#ur:end

def ur_joint_distances(current, target):
    return [
        get_joint_distance(current[0], target[0]),
        get_joint_distance(current[1], target[1]),
        get_joint_distance(current[2], target[2]),
        get_joint_distance(current[3], target[3]),
        get_joint_distance(current[4], target[4]),
        get_joint_distance(current[5], target[5])
    ]
#ur:end

# finds the joint that requires the longest time to reach the target position
def ur_get_leading_dim(distances, speeds, max_speed, max_acc, final_speed):
    i = 0
    itime = 10000.
    litime = 0.
    while i < len(distances):
        dist = distances[i]
        if fabs(dist) > UR_EPSILON:
            sign = fabs(dist) / dist
            dist = sign * dist  #fabs
            speed = speeds[i]
            if speed * sign < 0: # moving in the wrong direction
                extra = speed*speed / max_acc # add the (double) distance this joint will travel before it can reverse
                dist = dist + extra
            #ur:end
            speed_delta = fabs(speed) - final_speed
            if speed_delta > 0 and (dist <= 0.5 * speed_delta * speed_delta / max_acc):
                sign = -sign # we are close to the final goal and need to decelerate
            #ur:end
            speed = fabs(speed + sign * max_acc * UR_TIME_SLICE)
            if speed > max_speed:
                speed = max_speed
            #ur:end
            # this is (the inverse of) how long it would take to move to target given the updated speed
            tmp_itime = speed / dist
            if tmp_itime < itime:
                itime = tmp_itime
                litime = itime
            #ur:end
        #ur:end
        i = i + 1
    #ur:end
    return litime
#ur:end

# joint-linear motion
def ur_speed_joint_linear(target, max_speed, max_acc, final_speed):
    positions = get_target_joint_positions()
    speeds = get_target_joint_speeds()
    distances = ur_joint_distances(positions, target)
    itime = ur_get_leading_dim(distances, speeds, max_speed, max_acc, final_speed)
    return [
        distances[0] * itime,
        distances[1] * itime,
        distances[2] * itime,
        distances[3] * itime,
        distances[4] * itime,
        distances[5] * itime
    ]
#ur:end

# state globals
ctrl_last_cmd_id = 0
ctrl_last_cmd_time = 0
ctrl_last_cmd = UR_ZERO
ctrl_last_loop_time = 0
ctrl_is_contact = False
ctrl_is_moving = False
ctrl_is_deadman = False

# verifies that the control loop runs fast enough.
def ur_check_loop_delay(loop_time):
    time = ur_get_time()
    delay = time - loop_time
    if delay >= UR_TIME_SLICE*3:
        textmsg("drive loop delayed: ", delay)
    #ur:end
    return time
#ur:end

# Generate a target speed based on the latest command and current state
# Note that this is called in a loop on the drive thread by the real robot
def ur_get_target_speed(cmd_time, id, kind, target, max_speed, max_acc, force_low_bound, force_high_bound, controller, controller_args):
    # verify we are not running behind
    global ctrl_last_loop_time
    ctrl_last_loop_time = ur_check_loop_delay(ctrl_last_loop_time)
    # determine external forces and motion status
    force_limit_reached = ur_force_limit_exceeded(force_low_bound, force_high_bound)
    global ctrl_last_cmd_time
    global ctrl_is_contact
    # keep the is_contact flag on until we receive another command (to make sure we reported it back to the client)
    # cmd_time is the time we received the current command via the interface
    ctrl_is_contact = force_limit_reached or (ctrl_is_contact and (cmd_time == ctrl_last_cmd_time))
    ctrl_last_cmd_time = cmd_time
    global ctrl_last_cmd_id
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
    elif kind == UR_CMD_KIND_MOVE_JOINT_SPEEDS:
        cmd = target
        acc = max_acc
    elif kind == UR_CMD_KIND_MOVE_JOINT_POSITIONS: # this covers UR_CMD_KIND_MOVE_TOOL_POSE too, see interface.py
        final_speed = fabs(controller_args)
        cmd = ur_speed_joint_linear(target, max_speed, max_acc, final_speed)
        acc = max_acc
    #ur:end

    global ctrl_last_cmd
    if (norm(cmd) < UR_SPEED_NORM_ZERO) and (norm(cmd) <= norm(ctrl_last_cmd)):
        cmd = UR_ZERO
        acc = UR_FAST_STOP_ACCELERATION
    #ur:end

    global ctrl_is_moving
    ctrl_is_moving = norm(cmd) > UR_SPEED_NORM_ZERO or norm(get_target_joint_speeds()) > UR_SPEED_NORM_ZERO

    # update state
    ctrl_last_cmd = cmd
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
    return [cmd[0],
            cmd[1],
            cmd[2],
            cmd[3],
            cmd[4],
            cmd[5],
            acc,
            ctrl_last_cmd_time,
            ctrl_last_cmd_id,
            ctrl_last_loop_time,
            is_moving,
            is_contact,
            is_deadman]
#ur:end
