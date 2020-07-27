import numpy as np
from .hand import *

class HandController(object):
    '''
    This is the lowest level controller, communicating directly with the hand.
    There should be only one instance (per connection/hand), and all controller chains targeting the same hand must include this instance.
    '''
    def __init__(self, connection):
        self.connection = connection
        self.lastcmd = Command()
        self.readcmd = Command()

    def __call__(self, cmd, state):
        if not np.array_equal(cmd, self.lastcmd):
            self.connection.send(cmd, state)
            self.lastcmd[:] = cmd
        else:
            self.connection.execute(self.readcmd, state)
        return state