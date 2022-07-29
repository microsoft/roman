from .constants import *
from .urlib import *
from .control import *
from .drive import *

################################################################################################################################
## interface.py
## Implements the main entry point for clients, execute()
################################################################################################################################

# Builds the complete state response returned to clients after each command.
def get_arm_state(target_pose, target_joints):
    s = ur_get_status()
    # the order below mimics the UR's order (as defined by its RT interface))
    q = get_actual_joint_positions()
    qd = get_actual_joint_speeds()
    tp = get_actual_tcp_pose()
    tv = get_actual_tcp_speed()

    q_t = target_joints #get_target_joint_positions()
    qd_t = get_target_joint_speeds()
    tp_t = target_pose #get_target_tcp_pose()
    tv_t = get_target_tcp_speed()

    f = get_tcp_force()
    m = get_joint_torques()
    a = ur_get_tcp_acceleration()

    if UR_ROBOT_VERSION == UR_ROBOT_VERSION_CB2:
        ft = ur_get_tcp_sensor_force(True)  # FT sensor reading
    else:
        ft = f # on e-series, the ft sensor is built in and the value reported by get_tcp_force()
    #ur:end

    return [s[0], # read time
            s[1], # cmd_id
            0 + s[3] * UR_STATUS_FLAG_MOVING + s[4] * UR_STATUS_FLAG_CONTACT + s[5] * UR_STATUS_FLAG_DEADMAN, #status

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

def execute_arm_command(cmd, offset):
    kind = cmd[UR_CMD_KIND + offset]

    # ESTOP
    if kind == UR_CMD_KIND_ESTOP or kind >= UR_CMD_KIND_INVALID:
        ur_drive(ur_get_time(), 0, UR_CMD_KIND_MOVE_JOINT_SPEEDS, UR_ZERO, 0, 10, UR_FORCE_IGNORE_LOW, UR_FORCE_IGNORE_HIGH, UR_CMD_MOVE_CONTROLLER_DEFAULT, 0)
        return get_arm_state(UR_ZERO, UR_ZERO)
    #ur:end

    # READ
    if kind == UR_CMD_KIND_READ:
        return get_arm_state(UR_ZERO, UR_ZERO)
    #ur:end

    # IK_QUERY
    if kind == UR_CMD_KIND_IK_QUERY:
        target = s_(cmd, UR_CMD_MOVE_TARGET, offset)
        joint_target = get_inverse_kin(ur_pose(target))
        return get_arm_state(target, joint_target)
    #ur:end

    # CONFIG: payload (kg), tool center of gravity (vec3), tool tip (vec6)
    if kind == UR_CMD_KIND_CONFIG:
        set_payload(cmd[UR_CMD_CONFIG_MASS + offset], s_(cmd, UR_CMD_CONFIG_TOOL_COG, offset))
        set_tcp(ur_pose(s_(cmd, UR_CMD_CONFIG_TOOL_TIP, offset)))
        # todo: add support for setting workspace bounds
        # todo: add support for setting speed, acc and force bounds
        return get_arm_state(UR_ZERO, UR_ZERO)
    #ur:end

    # MOVE_XXX
    # When running on the real robot, the move command simply updates global variables that are picked up by the background drive thread.
    # Thus, the state of the arm does not reflect the command effects until the next cycle.
    # Rather than blocking for a full cycle, we simply return the previous state.
    # Note that the state vector contains the timestamp of the last command executed, and thus correctly reflects this behavior.
    id = cmd[UR_CMD_ID + offset]
    time = ur_get_time()
    target = s_(cmd, UR_CMD_MOVE_TARGET, offset)
    max_acceleration = cmd[UR_CMD_MOVE_MAX_ACCELERATION + offset]
    force_low_bound = s_(cmd, UR_CMD_MOVE_FORCE_LOW_BOUND, offset)
    force_high_bound = s_(cmd, UR_CMD_MOVE_FORCE_HIGH_BOUND, offset)
    controller = cmd[UR_CMD_MOVE_CONTROLLER + offset]
    controller_args = cmd[UR_CMD_MOVE_CONTROLLER_ARGS + offset]
    # all other controllers are implemented client-side for now, so replace with default controller
    if controller != UR_CMD_MOVE_CONTROLLER_DEFAULT:
        controller = UR_CMD_MOVE_CONTROLLER_DEFAULT
        controller_args = 0
    #ur:end
    if kind == UR_CMD_KIND_MOVE_TOOL_POSE:
        # convert tool pose to joints position
        kind = UR_CMD_KIND_MOVE_JOINT_POSITIONS
        #textmsg("target position",target_position)
        pose_target = target
        joint_target = get_inverse_kin(ur_pose(target))
        target = joint_target
        #textmsg("joint position",target_position)
    else:
        joint_target = target

        if UR_ROBOT_VERSION == UR_ROBOT_VERSION_ESERIES:
            pose_target = get_forward_kin(target)
        else:
            pose_target = UR_ZERO
        #ur:end
    #ur:end
    if UR_ROBOT_VERSION == UR_ROBOT_VERSION_ESERIES:
        sync()
    #ur:end
    max_speed = cmd[UR_CMD_MOVE_MAX_SPEED + offset]
    ur_drive(time, id, kind, target, max_speed, max_acceleration, force_low_bound, force_high_bound, controller, controller_args)
    return get_arm_state(pose_target, joint_target)
#ur:end

