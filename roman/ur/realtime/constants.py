################################################################################################################################
## constants.py
## Contains various constants and defaults used both on the urscript side and the python side
## The defaults can be oveeriden when calling utils.load_script, by including them in the defs hashtable.
################################################################################################################################
UR_ZERO = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
UR_STATE_ENTRIES_COUNT = 72 # This is how many numbers we expect to receive with every response
UR_PROTOCOL_VERSION = 0.1

# Possible types of commands
UR_CMD_KIND_ESTOP = 0
UR_CMD_KIND_MOVE_JOINT_SPEEDS = 1 # Accelerate to and maintain the specified speed
UR_CMD_KIND_MOVE_TOOL_POSE = 2 # Move towards an absolute goal position expressed as a tool pose.
UR_CMD_KIND_MOVE_JOINT_POSITIONS = 3 # Move towards an absolute goal position expressed in joint angles.
UR_CMD_KIND_READ = 8
UR_CMD_KIND_CONFIG = 9
UR_CMD_KIND_IK_QUERY = 10
UR_CMD_KIND_INVALID = 11

# Command field indices
UR_CMD_ID = 0
UR_CMD_TIME = 1
UR_CMD_KIND = 2
UR_CMD_CONFIG_MASS = 3
UR_CMD_CONFIG_TOOL_COG = [4, 7]
UR_CMD_CONFIG_TOOL_TIP = [7, 13]
UR_CMD_MOVE_TARGET = [3, 9]
UR_CMD_MOVE_MAX_SPEED = 9
UR_CMD_MOVE_MAX_ACCELERATION = 10
UR_CMD_MOVE_FORCE_LOW_BOUND = [11, 17]
UR_CMD_MOVE_FORCE_HIGH_BOUND = [17, 23]
UR_CMD_MOVE_CONTROLLER = 23
UR_CMD_MOVE_CONTROLLER_ARGS = 24
UR_CMD_ENTRIES_COUNT = 25 # This is how many floats we expect with each command (not including the count prefix). Needs to stay under 30 (UR restriction).

# Supported controllers
UR_CMD_MOVE_CONTROLLER_DEFAULT = 0 # default controller. In pose/position control, a low-bound on the final speed of the leading joint can be specified via controller_args.
UR_CMD_MOVE_CONTROLLER_TOUCH = 1 # expects to touch an obstacle and stop prior to eaching the specified target.
UR_CMD_MOVE_CONTROLLER_RT = 2 # reach the target in the specified amount of time (specified by UR_CMD_MOVE_TIME)

# State field indices
UR_STATE_TIME = 0
UR_STATE_CMD_ID = 1
UR_STATE_STATUS = 2
UR_STATE_JOINT_POSITIONS = [3, 9]
UR_STATE_JOINT_SPEEDS = [9, 15]
UR_STATE_TOOL_POSE = [15, 21]
UR_STATE_TOOL_SPEED = [21, 27]
UR_STATE_TARGET_JOINT_POSITIONS = [27, 33]
UR_STATE_TARGET_JOINT_SPEEDS = [33, 39]
UR_STATE_TARGET_TOOL_POSE = [39, 45]
UR_STATE_TARGET_TOOL_SPEED = [45, 51]
UR_STATE_JOINT_TORQUES = [51, 57]
UR_STATE_TOOL_FORCE = [57, 63]
UR_STATE_TOOL_ACCELERATION = [63, 66]
UR_STATE_SENSOR_FORCE = [66, 72]

# status bits
UR_STATUS_FLAG_MOVING = 1
UR_STATUS_FLAG_CONTACT = 2
UR_STATUS_FLAG_DEADMAN = 4
UR_STATUS_FLAG_RESERVED = 8
UR_STATUS_FLAG_DONE = 16
UR_STATUS_FLAG_GOAL_REACHED = 32
################################################################################################################################
## Default values
################################################################################################################################

# robot settings
UR_DEFAULT_MASS = 3.25
UR_DEFAULT_TOOL_COG = [0, 0, 0.12]
UR_DEFAULT_TCP = [0, 0, 0.12, 0, 0, 0]

# control
UR_TIME_SLICE = 1./125 # by default, use the CB2 version. The value gets overriden at runtime on higher UR versions (eseries).
UR_SPEED_TOLERANCE = 0.05 # rad/s # for ur5e, this worked better: UR_SPEED_TOLERANCE = 0.005 # rad/s
UR_SPEED_NORM_ZERO = 0.005 # rad/s
UR_JOINTS_POSITION_TOLERANCE = 0.001 # rad
UR_TOOL_POSITION_TOLERANCE = 0.001 # m
UR_TOOL_ROTATION_TOLERANCE = 0.005 # rad
UR_DEADMAN_SWITCH_LIMIT = 0.1 # seconds
UR_EPSILON = 0.00001

UR_DEFAULT_FORCE_LOW_BOUND = [-20.0, -20.0, -20.0, -2, -2, -2]
UR_DEFAULT_FORCE_HIGH_BOUND = [20.0, 20.0, 20.0, 2, 2, 2]
UR_FORCE_IGNORE_HIGH = [1000, 1000, 1000, 100, 100, 100]
UR_FORCE_IGNORE_LOW = [-1000, -1000, -1000, -100, -100, -100]
UR_DEFAULT_CONTACT_ALPHA = 0.5

UR_DEFAULT_ACCELERATION = 0.1 # rad/s2
UR_CONTACT_STOP_ACCELERATION = 50.0 # rad/s2
UR_FAST_STOP_ACCELERATION = 3.0 # rad/s2
UR_DEFAULT_MAX_SPEED = 0.1  # rad/s

# interface / protocol
UR_RT_PORT = 30003 # real-time UR interface (RT)
UR_ROBOT_IP = "192.168.1.2"
UR_DEFAULT_CLIENT_IP = "192.168.1.9"
UR_DEFAULT_CLIENT_PORT = 50003
UR_ROBOT_VERSION_CB2 = 0
UR_ROBOT_VERSION_ESERIES = 2
UR_ROBOT_VERSION = UR_ROBOT_VERSION_CB2

# these need to be defined outside, e.g. through the defs parameter of utils.load_script()
#UR_CLIENT_IP
#UR_CLIENT_PORT

# URScript-compatible slicing function
def s_(vec, bounds, start):
    s = bounds[0] + start
    cnt = bounds[1] - bounds[0]
    if cnt == 3:
        return [vec[s], vec[s + 1], vec[s + 2]]
    elif cnt == 6:
        return [vec[s], vec[s + 1], vec[s + 2], vec[s + 3], vec[s + 4], vec[s + 5]]
    #ur:end
#ur:end
