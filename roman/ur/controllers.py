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
        if cmd.kind() > UR_CMD_KIND_CONFIG:
            raise Exception(f"Invalid command: {cmd.id()}")
        self.connection.execute(cmd, state)
        if cmd.is_move_command():
            at_goal = cmd._goal_reached(state)
            state._set_state_flag(State._STATUS_FLAG_GOAL_REACHED, at_goal)
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

class TouchController:
    '''
    Expects contact before completing the motion. Verifies that the contact is not spurrious before assuming the goal is reached.
    '''
    def __init__(self, next):
        self.next = next
        self.contact_position = Joints()
        self.count = 1
        self.validation_count = 1
        self.cmd_id = 0
        self.force_sum = np.zeros(6)

    def execute(self, cmd, state):

        self.next.execute(cmd, state)

        if state.is_goal_reached():
            # stopped because the arm reached the goal but didn't detect contact, so this is a failure
            state._set_state_flag(State._STATUS_FLAG_GOAL_REACHED, 0)
            state._set_state_flag(State._STATUS_FLAG_DONE, 1)

        if cmd.id() != self.cmd_id and cmd.is_move_command():
            # new command, reset
            self.cmd_id = cmd.id()
            self.count = self.validation_count = cmd.contact_handling()
            self.force_sum[:] = 0
            return state

        if state.is_moving() and not state.is_contact():
            return state

        if self.count == 0 or np.any(self.force_sum < cmd.force_low_bound()*cmd.contact_handling()) or np.any(self.force_sum > cmd.force_high_bound()*cmd.contact_handling()):
            state._set_state_flag(State._STATUS_FLAG_GOAL_REACHED, 1)
            state._set_state_flag(State._STATUS_FLAG_DONE, 1)
            return state

        if not state.is_contact():
            return state

        if self.contact_position.allclose(state.joint_positions()):
            self.force_sum += state.sensor_force()
            self.count -= 1
        else:
            self.count = self.validation_count
            self.force_sum[:] = 0
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
        self.controllers = [ema, touch]
        self.cmd = Command()

    def execute(self, cmd, state):
        if cmd.kind() != UR_CMD_KIND_READ:
            # ignore read commands and simply send the last move command (or config cmd). The timestamp/id of the command identifies it as old.
            self.cmd[:] = cmd
        controller = self.controllers[int(self.cmd.controller_flags())]
        return controller.execute(self.cmd, state)
