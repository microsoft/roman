from . import constants
from . import urlib
from . import control
from . import drive

################################################################################################################################
## interface.py
## Implements the main entry point for clients, execute()
################################################################################################################################

# Builds the complete state response returned to clients after each command.
# !!! When running on UR, this function must be called from within a critical section.
def get_arm_state():
    s = control.ur_get_status()
    # the order below mimics the UR's order (as defined by its RT interface))
    q = urlib.get_actual_joint_positions()
    qd = urlib.get_actual_joint_speeds()
    tp = urlib.get_actual_tcp_pose()
    tv = urlib.get_actual_tcp_speed()
  
    q_t = urlib.get_target_joint_positions()
    qd_t = urlib.get_target_joint_speeds()
    tp_t = urlib.get_target_tcp_pose()
    tv_t = urlib.get_target_tcp_speed()
    
    f = urlib.get_tcp_force()
    m = urlib.get_joint_torques() 
    a = urlib.ur_get_tcp_acceleration()

    if constants.UR_ROBOT_VERSION == constants.UR_ROBOT_VERSION_CB2:
        ft = urlib.ur_get_tcp_sensor_force(True) #FT sensor reading
    else:
        ft = f # on e-series, the ft sensor is built in and the value reported by get_tcp_force()
    #ur:end

    
    return [s[0], # time
            s[1], # cmd_id
            0 + s[2] * constants.UR_STATUS_FLAG_MOVING + s[3] * constants.UR_STATUS_FLAG_CONTACT + s[4] * constants.UR_STATUS_FLAG_DEADMAN, #status

            q[0], q[1], q[2], q[3], q[4], q[5], 
            qd[0], qd[1], qd[2], qd[3], qd[4], qd[5], 
            tp[0], tp[1], tp[2], tp[3], tp[4], tp[5], 
            tv[0], tv[1], tv[2], tv[3], tv[4], tv[5], 
            
            q_t[0], q_t[1], q_t[2], q_t[3], q_t[4], q_t[5], 
            qd_t[0], qd_t[1], qd_t[2], qd_t[3], qd_t[4], qd_t[5], 
            tp_t[0], tp_t[1], tp_t[2], tp_t[3], tp_t[4], tp_t[5], 
            tv_t[0], tv_t[1], tv_t[2], tv_t[3], tv_t[4], tv_t[5],
            
            m[0], m[1], m[2], m[3], m[4], m[5], 
            f[0], f[1], f[2], f[3], f[4], f[5], 
            a[0], a[1], a[2],
            ft[0], ft[1], ft[2], ft[3], ft[4], ft[5]            
            ]
#ur:end

# !!! When running on UR, this function must be called from within a critical section.
def execute_arm_command(cmd, offset):
    kind = cmd[constants.UR_CMD_KIND + offset]
    
    # When running on the real robot, the move command simply updates global variables that are picked up by the background drive thread.
    # Thus, the state of the arm does not reflect the command effects until the next cycle.
    # Rather than blocking for a full cycle, we simply return the previous state. 
    # To make that obvious (and enforce it in the sim case), we explicitly capture the state first. 
    # Note that the state vector contains the timestamp of the last command executed, and thus correctly reflects this behavior.
    state = get_arm_state() 

    # READ
    if kind == constants.UR_CMD_KIND_READ:
        return state
    #ur:end

    # CONFIG: count, cmd code, payload (kg), tool center of gravity (vec3), tool tip (vec6), reset F/T sensor 
    if kind == constants.UR_CMD_KIND_CONFIG:
        urlib.set_payload(cmd[constants.UR_CMD_CONFIG_MASS + offset], constants.s_(cmd, constants.UR_CMD_CONFIG_TOOL_COG, offset))
        urlib.set_tcp(urlib.ur_pose(constants.s_(cmd, constants.UR_CMD_CONFIG_TOOL_TIP, offset))) 
        # todo: add support for setting workspace bounds
        # todo: add support for setting speed, acc and force bounds 
        return state
    #ur:end

    # MOVE_XXX
    id = cmd[constants.UR_CMD_ID + offset]
    time = urlib.ur_get_time()
    target = constants.s_(cmd, constants.UR_CMD_MOVE_TARGET, offset)
    max_acceleration = cmd[constants.UR_CMD_MOVE_MAX_ACCELERATION + offset]
    force_low_bound = constants.s_(cmd, constants.UR_CMD_MOVE_FORCE_LOW_BOUND, offset)
    force_high_bound = constants.s_(cmd, constants.UR_CMD_MOVE_FORCE_HIGH_BOUND, offset)
    contact_handling = cmd[constants.UR_CMD_MOVE_CONTACT_HANDLING + offset]
    if kind == constants.UR_CMD_KIND_MOVE_TOOL_POSE:
        # convert tool pose to joints position
        kind = constants.UR_CMD_KIND_MOVE_JOINTS_POSITION
        #textmsg("target position",target_position)
        target = urlib.get_inverse_kin(urlib.ur_pose(target))
        #textmsg("joint position",target_position)
    #ur:end
    max_speed = cmd[constants.UR_CMD_MOVE_MAX_SPEED + offset]
    drive.ur_drive(time, id, kind, target, max_speed, max_acceleration, force_low_bound, force_high_bound, contact_handling)
    return state
#ur:end

