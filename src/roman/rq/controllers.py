import numpy as np
from .hand import *

class HandController(object):
    '''
    This is the lowest level controller, communicating directly with the hand.
    There should be only one instance (per connection/hand), and all controller chains targeting the same hand must include this instance.
    '''
    def __init__(self, connection):
        self.connection = connection
        self.state = State()
        #self.lastcmd = Command()

    def __call__(self, cmd):
        #if np.array_equals(cmd, self.lastcmd)
        self.connection.send(cmd, self.state)
        return self.state