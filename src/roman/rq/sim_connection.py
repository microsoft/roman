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

    def send(self, cmd, state):
        pass

 