from .hand import *

################################################################
## Simulated hand implementation
################################################################
class SimConnection:
    """Implements functionality to read and command the simulated hand, regardless of simulator."""
    def __init__(self, env):
        self.env = env

    def connect(self, activate=True):
        pass

    def disconnect(self):
        pass

    def execute(self, cmd, state):
        # translate and send the command
        if cmd[Command._KIND] == Command._CMD_KIND_READ:
            pass
        elif cmd[Command._KIND] == Command._CMD_KIND_STOP:
            self.env.hand.stop()
        elif cmd[Command._KIND] == Command._CMD_KIND_CHANGE:
            self.env.hand.set_mode(cmd[Command._MODE])
        else: #Command._CMD_KIND_MOVE
            if cmd[Command._FINGER] == Finger.All:
                self.env.hand.move(cmd[Command._POSITION], cmd[Command._SPEED], cmd[Command._FORCE])
            else:
                self.env.hand.move_finger(cmd[Command._FINGER], cmd[Command._POSITION], cmd[Command._SPEED], cmd[Command._FORCE]) 


        # prepare the state
        self.env.update() # note that the sim update is called twice, once here and once by the arm's sim_connection
        self.env.hand.read()
        state[State._TIME] = self.env.time()
        state[State._FLAGS] = State._FLAG_READY \
                            + State._FLAG_MOVING * self.env.hand.is_moving() \
                            + State._FLAG_OBJECT_DETECTED * self.env.hand.object_detected()
        
        state[State._MODE] = self.env.hand.mode()
        positions = self.env.hand.positions()
        state[State._POSITION_A] = positions[0]
        state[State._POSITION_B] = positions[1]
        state[State._POSITION_C] = positions[2]
        targets = self.env.hand.targets()
        state[State._TARGET_A] = targets[0]
        state[State._TARGET_B] = targets[1]
        state[State._TARGET_C] = targets[2]

        return state

 