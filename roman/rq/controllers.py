import numpy as np
import time
from .hand import *

class HandController:
    '''
    This is the lowest level controller, communicating directly with the hand.
    There should be only one instance (per connection/hand), and all controller chains targeting the same hand must include this instance.
    '''
    def __init__(self, connection):
        self.connection = connection
        self.last_execution_time = time.perf_counter()

    def execute(self, cmd, state):
        if time.perf_counter() - self.last_execution_time < 0.01:
            time.sleep(0.01) # throttle to the frequency that the hand can sustain
        self.connection.execute(cmd, state)
        self.last_execution_time = time.perf_counter()
        return state

class NoHandController:
    '''
    Use when a hand is not available
    '''
    def __init__(self, connection):
        pass

    def execute(self, cmd, state):
        state[State._FLAGS] = State._FLAG_READY
        return state