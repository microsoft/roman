from .realtime import interface, urlib

################################################################
## Simulated robot implementation
################################################################
class SimConnection(object):
    """Implements functionality to read simulated robot state (arm and F/T sensor) and command the robot in real-time."""
    def __init__(self, env):
        self.env = env
        urlib.sim = env

    def connect(self):

        print('System ready.')

    def disconnect(self):
        pass

    def execute(self, cmd, state):
        """Sends the command to control layer and reads back the state, emulating the wire protocol used with the real robot."""
        state[:] = interface.execute_arm_command(cmd, 0)
        self.env.update()

 