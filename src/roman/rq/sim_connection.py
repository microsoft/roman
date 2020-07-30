from .hand import *

################################################################
## Simulated hand implementation
################################################################
class SimConnection(object):
    """Implements functionality to read and command the simulated hand."""
    def __init__(self, env):
        self.env = env

    def connect(self):
        pass

    def disconnect(self):
        pass

    def execute(self, cmd, state):
        state[State._TIME] = self.env.time()
        state[State._FLAGS] = State._FLAG_READY
        return state

 