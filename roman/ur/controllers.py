'''
Module containing various arm controllers.
Controllers transform higher-level commands into lower-level commands that can be executed by the arm or by lower level controllers.
Controllers can be chained together e.g. linear motion -> touch -> ur5
Each controller is initialized with the next controller in its chain, and is required to call it.
There can be multiple active controller chains, but only one can be used at a time.
'''

import numpy as np
import math
from .arm import *
from .realtime.constants import *

class BasicController:
    '''
    This is the lowest level controller, communicating directly with the arm.
    There should be only one instance (per connection/arm), and all controller chains targeting the same arm must include this instance.
    '''
    def __init__(self, connection):
        self.connection = connection

    def execute(self, cmd, state):
        if cmd.kind() >= UR_CMD_KIND_INVALID:
            raise Exception(f"Invalid command: {cmd.id()}")
        self.connection.execute(cmd, state)
        if cmd.is_move_command():
            at_goal = cmd._goal_reached(state)
            state._set_state_flag(State._STATUS_FLAG_GOAL_REACHED, at_goal)
            if cmd.kind() != UR_CMD_KIND_MOVE_JOINT_SPEEDS and cmd.controller() == UR_CMD_MOVE_CONTROLLER_DEFAULT and cmd.controller_args() == 0:
                state._set_state_flag(State._STATUS_FLAG_DONE, at_goal and not state.is_moving())
            else:
                state._set_state_flag(State._STATUS_FLAG_DONE, at_goal)
        return state

class EMAForceCalibrator:
    '''
    Keeps an exponentially weighted moving average of the force reported by the FT sensor.
    Adds the average to the expected force of each move command.
    Substracts the average from the reported sensor force of each response.
    avg(t) = alpha*sample(t) + (1-alpha)*avg(t-1)
    '''
    def __init__(self, next, alpha = 0.01):
        self.next = next
        self.alpha = alpha
        self.sample = np.zeros(6)
        self.force_average = np.zeros(6) # we are assuming the FT sensor is reset to zero on startup
        self.cmd = Command()

    def execute(self, cmd, state):
        self.cmd[:] = cmd
        if cmd.is_move_command():
            np.add(self.force_average, cmd.force_low_bound(), self.cmd.force_low_bound().array)
            np.add(self.force_average, cmd.force_high_bound(), self.cmd.force_high_bound().array)

        self.next.execute(self.cmd, state)
        if not state.is_contact():
            self.sample[:] = state.sensor_force()
            np.multiply(self.force_average, 1-self.alpha, self.force_average)
            np.multiply(self.sample, self.alpha, self.sample)
            np.add(self.force_average, self.sample, self.force_average)
        # else:
        #     print(f"Contact detected:{state.sensor_force()}, outside of bounds {self.cmd.force_low_bound()} and {self.cmd.force_high_bound()}")

        np.subtract(state.sensor_force(), self.force_average, state.sensor_force().array)
        return state

class IncrementalController:
    '''
    This controller moves the arm in the direction of the target pose for the specified amount
    of time. The last portion of the trajectory is executed open-loop, to avoid hunting for the
    target position. The final speed is unspecified, and is derived from the other motion constraints.
    It is assumed that the client issues new commands with relatively high frequency (e.g. 30Hz),
    providing vision-based corrections to reach some higher-level goal.
    '''
    def __init__(self, next, open_loop_duration=0.008):
        self.next = next
        self.open_loop_duration = open_loop_duration
        self.cmd = Command()
        self.cmd_read = Command()
        self.start_time = 0
        self.target = None
        self.done = 0
        self.goal_reached = 0

    def execute(self, cmd, state):
        if cmd.kind() != UR_CMD_KIND_MOVE_JOINT_POSITIONS and cmd.kind() != UR_CMD_KIND_MOVE_TOOL_POSE:
            self.next.execute(cmd, state)
            return state

        if self.cmd.id() != cmd.id():
            # new command
            self.cmd[:] = cmd
            self.cmd_read[:] = cmd
            self.cmd_read.make_read() # this preserves the id of the initial command
            self.next.execute(cmd, state)
            self.start_time = state.time()
            self.target = state.target_joint_positions()
            self.done = 0
            self.goal_reached = 0
            return state

        if self.done:
            # read state but stop sending speed commands
            self.next.execute(self.cmd_read, state)
            state._set_state_flag(State._STATUS_FLAG_GOAL_REACHED, self.goal_reached)
            state._set_state_flag(State._STATUS_FLAG_DONE, self.done)
            return state

        self.next.execute(self.cmd, state)
        time_left = cmd.controller_args() - state.time() + self.start_time
        self.done = time_left <= 0
        self.goal_reached = cmd._goal_reached(state)
        if time_left > self.open_loop_duration:
            # prep the speed command
            # speed derived from: d = t*(vf+v0)/2 => vf = 2d/t - v0
            speed = (self.target - state.joint_positions()) * 2 / time_left - state.joint_speeds()
            speed = np.where(np.fabs(speed) > UR_SPEED_TOLERANCE, speed, 0)
            if np.amax(np.fabs(speed)) > self.cmd.max_speed():
                speed = speed * (self.cmd.max_speed() / np.amax(np.fabs(speed)))
            self.cmd[Command._KIND] = UR_CMD_KIND_MOVE_JOINT_SPEEDS
            self.cmd[Command._MOVE_TARGET] = speed
            acc = np.amax(np.fabs(speed - state.joint_speeds())) / time_left
            self.cmd[Command._MOVE_MAX_ACCELERATION] = min(acc, self.cmd.max_acceleration())

        state._set_state_flag(State._STATUS_FLAG_GOAL_REACHED, self.goal_reached)
        state._set_state_flag(State._STATUS_FLAG_DONE, self.done)
        return state

class TouchController:
    '''
    Expects contact before completing the motion. Verifies that the contact is not spurrious before assuming the goal is reached.
    '''
    def __init__(self, next):
        self.next = next
        self.contact_position = Joints()
        self.contact_counter = 1
        self.validation_count = 1
        self.cmd_id = 0
        self.force_sum = np.zeros(6)
        self.done = 0
        self.goal_reached = 0

    def execute(self, cmd, state):

        if self.done and cmd.id() == self.cmd_id:
            self.next.execute(self.cmd_stop, state)
            state._set_state_flag(State._STATUS_FLAG_GOAL_REACHED, self.goal_reached)
            state._set_state_flag(State._STATUS_FLAG_DONE, self.done)
            return state
        else:
            self.next.execute(cmd, state)

        if cmd.id() != self.cmd_id and cmd.is_move_command():
            # new command, reset
            self.cmd_id = cmd.id()
            self.cmd_stop = cmd.clone().make_estop() # this preserves the id
            self.contact_counter = self.validation_count = cmd.controller_args()
            self.force_sum[:] = 0
            self.done = 0
            self.goal_reached = 0

        if state.is_goal_reached():
            # the arm reached the position goal but didn't detect contact, so this is a failure, and goal_reached remains False
            self.goal_reached = 0
            self.done = 1
            state._set_state_flag(State._STATUS_FLAG_GOAL_REACHED, self.goal_reached)
            state._set_state_flag(State._STATUS_FLAG_DONE, self.done)
            return state

        if state.is_moving() and not state.is_contact():
            return state

        if np.any(self.force_sum < cmd.force_low_bound()*cmd.controller_args()) or np.any(self.force_sum > cmd.force_high_bound()*cmd.controller_args()):
            self.goal_reached = 1
            self.done = 1
            state._set_state_flag(State._STATUS_FLAG_GOAL_REACHED, self.goal_reached)
            state._set_state_flag(State._STATUS_FLAG_DONE, self.done)
            return state

        if not state.is_contact():
            return state

        if self.contact_position.allclose(state.joint_positions()):
            # we hit something that doesn't budge
            self.force_sum += state.sensor_force()
            self.contact_counter -= 1
        else:
            # we encountered an obstacle for the first time, or the obstacle moved, or the previous spikes were just noise
            self.contact_counter = self.validation_count - 1 
            self.force_sum[:] = state.sensor_force()
            self.contact_position[:] = state.joint_positions()

        return state


class ArmController:
    '''
    This is the highest-level controller.
    Its job is to determine which controller hierarchy to set up for a given command.
    '''
    def __init__(self, connection):
        basic = BasicController(connection)
        ema = EMAForceCalibrator(basic)
        touch = TouchController(ema)
        inc = IncrementalController(ema)
        self.controllers = {
            UR_CMD_MOVE_CONTROLLER_DEFAULT: ema,
            UR_CMD_MOVE_CONTROLLER_TOUCH: touch,
            UR_CMD_MOVE_CONTROLLER_RT: inc}

    def execute(self, cmd, state):
        controller = self.controllers[int(cmd.controller())]
        return controller.execute(cmd, state)
