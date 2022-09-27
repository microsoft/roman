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
        self.lastcmd = Command()
        self.readcmd = Command()

    def execute(self, cmd, state):
        if not np.array_equal(cmd, self.lastcmd): 
            self.connection.execute(cmd, state)
            self.lastcmd[:] = cmd
            time.sleep(0.1)

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