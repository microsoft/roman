from robot.simenv import SimEnvironment
import robot.URScripts.interface as interface

################################################################
## Simulated robot implementation
################################################################
class SimURConnection(object):
    """Implements functionality to read simulated robot state (arm and F/T sensor) and command the robot in real-time."""
    def connect(self):
        self.env = SimEnvironment()
        self.env.reset()
        self.env.update()
        print('System ready.')

    def disconnect(self):
        pass

    def send(self, cmd, state):
        """Sends the command to control layer and reads back the state, emulating the wire protocol used with the real robot."""
        state[:] = interface.execute_arm_command(cmd, 0)
        self.env.update()

 