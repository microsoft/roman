import numpy as np
import math
import time
from scipy.spatial.transform import Rotation
from ..common import Vec, equal_angle, clamp_angle
from .realtime.constants import *

class Joints(Vec):

    '''
    A vector of 6 elements representing joint properties. The order is: base, shoulder, elbow, wrist1, wrist2, wrist3.
    '''
    BASE = 0
    SHOULDER = 1
    ELBOW = 2
    WRIST1 = 3
    WRIST2 = 4
    WRIST3 = 5

    def __init__(self, base=0, shoulder=0, elbow=0, wrist1=0, wrist2=0, wrist3=0):
        super().__init__(6)
        self.array[:] = [base, shoulder, elbow, wrist1, wrist2, wrist3]

    def _fix(self):
        for i in range(6):
            self.array[i] = clamp_angle(self.array[i])

    def allclose(self, array, tolerance = UR_JOINTS_POSITION_TOLERANCE):
        if isinstance(array, Tool):
            raise TypeError("Cannot compare joint positions with tool pose.")

        if np.allclose(self.array, array, rtol=0, atol=tolerance):
            return True
        for i in range(len(array)):
            if not equal_angle(self.array[i], array[i], tolerance):
                return False
        return True

    def interpolate(self, target, fraction):
        return (target - self) * fraction

class JointSpeeds(Joints):
    pass

class Tool(Vec):
    '''
    A vector of 6 elements representing tool position and orientation. Orientation is stored as axis angles. The order is: x, y, z, rx, ry, rz.
    '''
    X = 0
    Y = 1
    Z = 2
    RX = 3
    RY = 4
    RZ = 5

    def __init__(self, x=0, y=0, z=0, rx=0, ry=0, rz=0):
        super().__init__(6)
        self.array[:] = [x, y, z, rx, ry, rz]

    def _fix(self):
        for i in range(3, 6):
            self.array[i] = clamp_angle(self.array[i])

    def allclose(self, array, position_tolerance = UR_TOOL_POSITION_TOLERANCE, rotation_tolerance = UR_TOOL_ROTATION_TOLERANCE):
        if isinstance(array, Joints):
            raise TypeError("Cannot compare tool pose with joint positions.")

        if not np.allclose(self.array[:3], array[:3], rtol=0, atol=position_tolerance):
            return False

        # normalize the two rotation vectors first
        a = Rotation.from_rotvec(self.array[3:6]).as_euler("xyz")
        b = Rotation.from_rotvec(array[3:6]).as_euler("xyz")
        return  equal_angle(a[0], b[0], rotation_tolerance) \
            and equal_angle(a[1], b[1], rotation_tolerance) \
            and equal_angle(a[2], b[2], rotation_tolerance)

    @staticmethod
    def from_xyzrpy(xyzrpy):
        r = Rotation.from_euler("xyz", xyzrpy[3:]).as_rotvec()
        return Tool.fromarray(np.concatenate((xyzrpy[:3], r)))

    def to_xyzrpy(self):
        r = Rotation.from_rotvec(self.array[3:]).as_euler("xyz")
        return np.concatenate((self.array[:3], r))

    def position(self):
        return self.array[:3]

    def orientation(self):
        return Rotation.from_rotvec(self.array[3:]).as_quat()

    def interpolate(self, target, fraction):
        current = self.to_xyzrpy()
        delta = Tool.fromarray(target.to_xyzrpy() - current)
        intermediate_target = delta * fraction + current
        return Tool.from_xyzrpy(intermediate_target)

class ToolSpeed(Tool):
    pass

################################################################
## Arm state.
################################################################
class State(Vec):
    '''
    The state of a 6-dof arm, encoded as a 72-valued vector.
    '''
    _BUFFER_SIZE = UR_STATE_ENTRIES_COUNT
    _TIME = UR_STATE_TIME
    _CMD_ID = UR_STATE_CMD_ID
    _STATUS = UR_STATE_STATUS
    _JOINT_POSITIONS = slice(*UR_STATE_JOINT_POSITIONS)
    _JOINT_SPEEDS =  slice(*UR_STATE_JOINT_SPEEDS)
    _TOOL_POSE =  slice(*UR_STATE_TOOL_POSE)
    _TOOL_SPEED =  slice(*UR_STATE_TOOL_SPEED)
    _TARGET_JOINT_POSITIONS =  slice(*UR_STATE_TARGET_JOINT_POSITIONS)
    _TARGET_JOINT_SPEEDS =  slice(*UR_STATE_TARGET_JOINT_SPEEDS)
    _TARGET_TOOL_POSE =  slice(*UR_STATE_TARGET_TOOL_POSE)
    _TARGET_TOOL_SPEED =  slice(*UR_STATE_TARGET_TOOL_SPEED)
    _TOOL_FORCE =  slice(*UR_STATE_TOOL_FORCE)
    _JOINT_TORQUES =  slice(*UR_STATE_JOINT_TORQUES)
    _TOOL_ACCELERATION =  slice(*UR_STATE_TOOL_ACCELERATION)
    _SENSOR_FORCE =  slice(*UR_STATE_SENSOR_FORCE)
    _STATUS_FLAG_MOVING = UR_STATUS_FLAG_MOVING
    _STATUS_FLAG_CONTACT = UR_STATUS_FLAG_CONTACT
    _STATUS_FLAG_DEADMAN = UR_STATUS_FLAG_DEADMAN
    _STATUS_FLAG_DONE = UR_STATUS_FLAG_DONE
    _STATUS_FLAG_GOAL_REACHED = UR_STATUS_FLAG_GOAL_REACHED

    def __init__(self):
        super().__init__(State._BUFFER_SIZE)

    def time(self):
        '''The time when the response was generated, in robot time'''
        return self[State._TIME]

    def cmd_id(self):
        '''The id of the last drive command, set to client time when command was submitted.'''
        return self[State._CMD_ID]

    def is_valid(self):
        '''True if the state represents an actual arm state, false if this is is an empty vector.'''
        return self[State._TIME] > 0

    def is_moving(self):
        '''True if the arm is stopped, false if the arm is moving.'''
        return int(self[State._STATUS]) & State._STATUS_FLAG_MOVING != 0

    def is_contact(self):
        ''' True if the arm is experiencing a force greater than threshold specified with the last move command.'''
        return int(self[State._STATUS]) & State._STATUS_FLAG_CONTACT != 0

    def is_deadman_switch_triggered(self):
        '''True if the arm stopped because of a deadman switch, false otherwise.'''
        return int(self[State._STATUS]) & State._STATUS_FLAG_DEADMAN != 0

    def is_goal_reached(self):
        ''' True if the arm reached the goal established by the last move command, False otherwise. The flag is meaningful only when is_done() is also True'''
        return int(self[State._STATUS]) & State._STATUS_FLAG_GOAL_REACHED != 0

    def is_done(self):
        ''' True if the arm completed the last move command.'''
        return int(self[State._STATUS]) & State._STATUS_FLAG_DONE != 0

    def joint_positions(self):
        '''The current actual joint angular position vector in rad : [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]'''
        return Joints.fromarray(self[State._JOINT_POSITIONS], False)

    def joint_speeds(self):
        '''The current actual joint angular velocity vector in rad/s: [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]'''
        return JointSpeeds.fromarray(self[State._JOINT_SPEEDS], False)

    def tool_pose(self):
        '''The current actual TCP vector : ([X, Y, Z, Rx, Ry, Rz])'''
        return Tool.fromarray(self[State._TOOL_POSE], False)

    def tool_speed(self):
        '''The current actual TCP velocity vector; ([X, Y, Z, Rx, Ry, Rz])'''
        return ToolSpeed.fromarray(self[State._TOOL_SPEED], False)

    def target_joint_positions(self):
        '''The current target joint angular position vector in rad: [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]'''
        return Joints.fromarray(self[State._TARGET_JOINT_POSITIONS], False)

    def target_joint_speeds(self):
        '''The current target joint angular velocity vector in rad/s: [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]'''
        return JointSpeeds.fromarray(self[State._TARGET_JOINT_SPEEDS], False)

    def target_tool_pose(self):
        '''The current target TCP vector; ([X, Y, Z, Rx, Ry, Rz])'''
        return Tool.fromarray(self[State._TARGET_TOOL_POSE], False)

    def target_tool_speed(self):
        '''The TCP speed. The ﬁrst three values are the cartesian speeds along x,y,z, and the last three deﬁne the current rotation axis, rx,ry,rz, and the length|rz,ry,rz|deﬁnes the angular velocity in radians/s'''
        return ToolSpeed.fromarray(self[State._TARGET_TOOL_SPEED], False)

    def tool_force(self):
       '''Returns the wrench (Force/Torque vector) at the TCP.
        The external wrench is computed based on the error between the joint torques required to stay on the trajectory and the expected joint torques.
        The function returns ”p[Fx (N), Fy(N), Fz(N), TRx (Nm), TRy (Nm), TRz (Nm)]”,
        where Fx, Fy, and Fx are the forces in the axes of the robot base coordinate system measured in Newtons, and TRx, TRy, and TRz are the torques around these axes measyred in Newton times Meters. '''
       return Tool.fromarray(self[State._TOOL_FORCE], False)

    def joint_torques(self):
        '''The torque on the joints, corrected by the torque needed to move the robot itself (gravity, friction, etc.), returned as a vector of length 6.'''
        return Joints.fromarray(self[State._JOINT_TORQUES], False)

    def tool_acceleration(self):
        '''The accelerometer reading, in the robot base coordinate system'''
        return self[State._TOOL_ACCELERATION]

    def sensor_force(self):
        '''The forces and moments reported by the Force/Torque sensor mounted on the wrist, in the axes of the robot base coordinate system'''
        return Tool.fromarray(self[State._SENSOR_FORCE], False)

    def _set_state_flag(self, flag, value):
        if value:
            self[State._STATUS] = int(self[State._STATUS]) | flag
        else:
            self[State._STATUS] = int(self[State._STATUS]) & (0xFFFFFFFF ^ flag)

################################################################
## Arm commands
################################################################
class Command(Vec):
    '''
    Represents a command that can be sent to the arm.
    The move target can be joint speeds, or an absolute position, specified either as a tool pose or as joint angles.
    In either case, the arm stops when contact is detected, that is, if fabs(actual_force - expected_force) > max_force.
    An optional motion controller choice can be specified using the controller and controler_args parameters.
    Other commands (read and config) can be initialized using the as_xxx methods.
    '''
    _BUFFER_SIZE = UR_CMD_ENTRIES_COUNT
    _ID = UR_CMD_ID
    _TIME = UR_CMD_TIME
    _KIND = UR_CMD_KIND
    _CONFIG_MASS = UR_CMD_CONFIG_MASS
    _CONFIG_TOOL_COG = slice(*UR_CMD_CONFIG_TOOL_COG)
    _CONFIG_TOOL_TIP = slice(*UR_CMD_CONFIG_TOOL_TIP)
    _MOVE_TARGET = slice(*UR_CMD_MOVE_TARGET)
    _MOVE_MAX_SPEED = UR_CMD_MOVE_MAX_SPEED
    _MOVE_MAX_ACCELERATION = UR_CMD_MOVE_MAX_ACCELERATION
    _MOVE_FORCE_LOW_BOUND = slice(*UR_CMD_MOVE_FORCE_LOW_BOUND)
    _MOVE_FORCE_HIGH_BOUND = slice(*UR_CMD_MOVE_FORCE_HIGH_BOUND)
    _MOVE_CONTROLLER = UR_CMD_MOVE_CONTROLLER
    _MOVE_CONTROLLER_ARGS = UR_CMD_MOVE_CONTROLLER_ARGS

    def __init__(self, id=0):
        super().__init__(Command._BUFFER_SIZE)
        self[Command._KIND]  = UR_CMD_KIND_READ
        self[Command._ID] = id

    def make(self,
            kind = UR_CMD_KIND_READ,
            target=UR_ZERO,
            max_speed=UR_DEFAULT_MAX_SPEED,
            max_acc=UR_DEFAULT_ACCELERATION,
            force_low_bound:Tool=UR_DEFAULT_FORCE_LOW_BOUND,
            force_high_bound:Tool=UR_DEFAULT_FORCE_HIGH_BOUND,
            controller=0,
            controller_args=0):

        self[Command._KIND] = kind
        self[Command._MOVE_TARGET] = target
        self[Command._MOVE_MAX_SPEED] = max_speed
        self[Command._MOVE_MAX_ACCELERATION] = max_acc
        self[Command._MOVE_FORCE_LOW_BOUND] = force_low_bound if force_low_bound is not None else UR_FORCE_IGNORE_LOW
        self[Command._MOVE_FORCE_HIGH_BOUND] = force_high_bound if force_high_bound is not None else UR_FORCE_IGNORE_HIGH
        self[Command._MOVE_CONTROLLER] = controller
        self[Command._MOVE_CONTROLLER_ARGS] = controller_args
        return self

    def make_read(self):
        return self.make(kind=UR_CMD_KIND_READ)

    def make_estop(self):
        return self.make(kind=UR_CMD_KIND_MOVE_JOINT_SPEEDS, max_acc=6, force_low_bound=None, force_high_bound=None)

    def id(self): return self[Command._ID]
    def time(self): return self[Command._TIME]
    def kind(self): return self[Command._KIND]
    def target(self):
            if self[Command._KIND] == UR_CMD_KIND_MOVE_TOOL_POSE:
                return Tool.fromarray(self[Command._MOVE_TARGET], False)
            elif self[Command._KIND] == UR_CMD_KIND_MOVE_JOINT_POSITIONS:
                return Joints.fromarray(self[Command._MOVE_TARGET], False)
            return JointSpeeds.fromarray(self[Command._MOVE_TARGET], False)
    def max_speed(self): return self[Command._MOVE_MAX_SPEED]
    def max_acceleration(self): return self[Command._MOVE_MAX_ACCELERATION]
    def force_low_bound(self): return Tool.fromarray(self[Command._MOVE_FORCE_LOW_BOUND], False)
    def force_high_bound(self): return Tool.fromarray(self[Command._MOVE_FORCE_HIGH_BOUND], False)
    def controller(self): return self[Command._MOVE_CONTROLLER]
    def controller_args(self): return self[Command._MOVE_CONTROLLER_ARGS]
    def is_move_command(self): return self[Command._KIND] > UR_CMD_KIND_ESTOP and self[Command._KIND] < UR_CMD_KIND_READ
    def _goal_reached(self, state):
        if state.cmd_id() != self.id():
            return False
        if self[Command._KIND] == UR_CMD_KIND_MOVE_JOINT_SPEEDS:
            return self.target().allclose(state.joint_speeds(), UR_SPEED_TOLERANCE)
        elif self[Command._KIND] == UR_CMD_KIND_MOVE_JOINT_POSITIONS:
            return self.target().allclose(state.joint_positions())
        elif self[Command._KIND] == UR_CMD_KIND_MOVE_TOOL_POSE:
            return self.target().allclose(state.tool_pose())
        else:
            raise Exception("Invalid command type")


class Arm:
    _READ_CMD = Command()

    def __init__(self, controller):
        self.controller = controller
        self.command = Command()
        self.state = State()
        self.last_cmd_id = 0

    def __execute(self, blocking):
        self.last_cmd_id += 1
        self.command[Command._ID] = self.last_cmd_id
        self.command[Command._TIME] = int(time.perf_counter()*1000)/1000
        self.controller.execute(self.command, self.state)
        assert self.state.cmd_id() != self.command.id()
        self.state[:] = 0 # invalidate the state, since it doesn't reflect the command yet. 

        # print(time.perf_counter()- start_time)
        while blocking and (self.state.cmd_id() != self.command.id() or not self.state.is_done()):
            self.step()

    def execute(self, command, blocking):
        self.command[:] = command
        self.__execute(blocking)

    def step(self):
        last_time = self.state.time()
        while last_time == self.state.time() or self.state.cmd_id() != self.command.id():
            self.controller.execute(self.command, self.state)

    def read(self):
        self.execute(Arm._READ_CMD, False)

    def get_inverse_kinematics(self, target_position):
        if type(target_position) is not Tool:
            raise TypeError('Argument target_position must be of type Tool.')
        self.command.make(
            kind = UR_CMD_KIND_IK_QUERY,
            target = target_position)
        self.__execute(blocking=False)
        self.step() # wait for the result
        return self.state.target_joint_positions().clone()

    def move(self,
            target_position,
            max_speed=UR_DEFAULT_MAX_SPEED,
            max_acc=UR_DEFAULT_ACCELERATION,
            force_low_bound=UR_DEFAULT_FORCE_LOW_BOUND,
            force_high_bound=UR_DEFAULT_FORCE_HIGH_BOUND,
            controller=0,
            controller_args=0,
            blocking=True):

        if type(target_position) is Joints:
            cmd_type = UR_CMD_KIND_MOVE_JOINT_POSITIONS
        elif type(target_position) is Tool:
            cmd_type = UR_CMD_KIND_MOVE_TOOL_POSE
        elif type(target_position) is JointSpeeds:
            cmd_type = UR_CMD_KIND_MOVE_JOINT_SPEEDS
        else:
            raise TypeError('Argument target_position must be of type Tool, Joints or JointSpeeds. '
                            'Use [Tool|Joints|JointSpeeds].fromarray() to wrap an existing array.')

        self.command.make(
            kind = cmd_type,
            target = target_position,
            max_speed=max_speed,
            max_acc=max_acc,
            force_low_bound=force_low_bound,
            force_high_bound=force_high_bound,
            controller=controller,
            controller_args=controller_args)
        self.__execute(blocking)

    def speed(self,
            target_speed,
            acc=UR_DEFAULT_ACCELERATION,
            force_low_bound=UR_DEFAULT_FORCE_LOW_BOUND,
            force_high_bound=UR_DEFAULT_FORCE_HIGH_BOUND,
            controller=0,
            controller_args=0,
            blocking=True):
        if type(target_speed) is not JointSpeeds:
            raise TypeError("Speed can only be specified as joint velocities. Argument target_speed must be of type JointSpeeds.")

        self.command.make(
            kind = UR_CMD_KIND_MOVE_JOINT_SPEEDS,
            target = target_speed,
            max_acc = acc,
            force_low_bound=force_low_bound,
            force_high_bound=force_high_bound,
            controller=0,
            controller_args=0)
        self.__execute(blocking)

    def stop(self, acc = UR_FAST_STOP_ACCELERATION, blocking = True):
        self.speed(target_speed = JointSpeeds(*UR_ZERO), acc=acc)

    def touch(self,
            target_position,
            max_speed=UR_DEFAULT_MAX_SPEED,
            max_acc=UR_DEFAULT_ACCELERATION,
            force_low_bound=[-5, -5, -5, -0.5, -0.5, -0.5],
            force_high_bound=[5, 5, 5, 0.5, 0.5, 0.5],
            max_final_speed=0,
            blocking=True):
        self.move(
            target_position,
            max_speed=max_speed,
            max_acc=max_acc,
            force_low_bound=force_low_bound,
            force_high_bound=force_high_bound,
            controller=UR_CMD_MOVE_CONTROLLER_TOUCH,
            controller_args=max_final_speed,
            blocking=blocking)

    def config(self, mass = UR_DEFAULT_MASS, cog = UR_DEFAULT_TOOL_COG, tcp = UR_DEFAULT_TCP):
        self.command.make()
        self.command[Command._KIND] = UR_CMD_KIND_CONFIG
        self.command[Command._CONFIG_MASS]=mass
        self.command[Command._CONFIG_TOOL_COG]=cog
        self.command[Command._CONFIG_TOOL_TIP]=tcp
        self.__execute(blocking=False)