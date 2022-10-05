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
            if cmd.controller() == UR_CMD_MOVE_CONTROLLER_DEFAULT and cmd.controller_args() == 0:
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
        self.sample = Joints()
        self.force_average = Joints() # we are assuming the FT sensor is reset to zero on startup
        self.cmd = Command()

    def execute(self, cmd, state):
        self.cmd[:] = cmd
        if cmd.is_move_command():
            np.add(self.force_average, cmd.force_low_bound(), self.cmd.force_low_bound().array)
            np.add(self.force_average, cmd.force_high_bound(), self.cmd.force_high_bound().array)

        self.next.execute(self.cmd, state)
        if not state.is_contact():
            self.sample[:] = state.sensor_force()
            np.multiply(self.force_average, 1-self.alpha, self.force_average.array)
            np.multiply(self.sample, self.alpha, self.sample.array)
            np.add(self.force_average, self.sample, self.force_average.array)
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
        self.cmd_read = Command().make()
        self.open_loop_duration = open_loop_duration
        self.cmd = None
        self.start_time = 0
        self.speed_cmd = None
        self.target = None

    def execute(self, cmd, state):
        if cmd.kind() != UR_CMD_KIND_MOVE_JOINT_POSITIONS and cmd.kind() != UR_CMD_KIND_MOVE_TOOL_POSE:
            self.next.execute(self.speed_cmd, state)
            return state

        if self.cmd != cmd:
            # new command
            self.cmd = cmd.clone()
            self.next.execute(cmd, state)
            self.start_time = state.time()
            self.target = state.target_joint_positions()
            self.speed_cmd = self.cmd.clone()
            self.speed_cmd[Command._KIND] = UR_CMD_KIND_MOVE_JOINT_SPEEDS
            return state

        time_left = self.cmd.controller_args() - state.time() + self.start_time
        if time_left <= 0:
            self.speed_cmd[Command._MOVE_TARGET] = UR_ZERO
            self.next.execute(self.speed_cmd, state)
        else:
            if time_left > self.open_loop_duration:
                # we are far enough to perform corrections
                self.next.execute(cmd, state)
                # prep the speed command
                # speed derived from: d = t*(vf+v0)/2 => vf = 2d/t - v0
                speed = (self.target - state.joint_positions()) * 2 / time_left - state.joint_speeds()
                speed = np.where(np.fabs(speed) > UR_SPEED_TOLERANCE, speed, 0)
                if np.amax(np.fabs(speed)) > self.cmd.max_speed():
                    speed = speed * (self.cmd.max_speed() / np.amax(np.fabs(speed)))
                self.speed_cmd[Command._MOVE_TARGET] = speed
                acc = np.amax(np.fabs(speed - state.joint_speeds())) / time_left
                self.speed_cmd[Command._MOVE_MAX_ACCELERATION] = min(acc, self.cmd.max_acceleration())
            else:
                # we are close to the time limit, no more corrections
                self.next.execute(self.speed_cmd, state)

        state._set_state_flag(State._STATUS_FLAG_GOAL_REACHED, cmd._goal_reached(state))
        self.cmd.controller_args() - state.time() + self.start_time
        state._set_state_flag(State._STATUS_FLAG_DONE, time_left <= 0)
        return state

class TouchController:
    '''
    Expects contact before completing the motion.
    '''
    def __init__(self, next):
        self.next = next

    def execute(self, cmd, state):
        self.next.execute(cmd, state)
        if state.cmd_id() != cmd.id():
            return state
        if state.is_contact():
            state._set_state_flag(State._STATUS_FLAG_GOAL_REACHED, 1)
            state._set_state_flag(State._STATUS_FLAG_DONE, 1)
        elif state.is_goal_reached():
            # stopped because the arm reached the goal but didn't detect contact, so this is a failure
            state._set_state_flag(State._STATUS_FLAG_GOAL_REACHED, 0)
            state._set_state_flag(State._STATUS_FLAG_DONE, 1)
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
        self.cmd = Command()

    def execute(self, cmd, state):
        self.cmd[:] = cmd
        controller = self.controllers[int(self.cmd.controller())]
        return controller.execute(self.cmd, state)
