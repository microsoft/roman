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
        self.__last = time.time()

    def execute(self, cmd, state):
        if not np.array_equal(cmd, self.lastcmd):
            self.connection.execute(cmd, state)
            self.lastcmd[:] = cmd
        elif time.time() - self.__last > 0.01: # max recommended hand freq
            self.connection.execute(self.readcmd, state)
            self.__last = time.time()
            
        return state