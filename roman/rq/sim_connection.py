from . import hand

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
        if cmd[hand.Command._KIND] == hand.Command._CMD_KIND_READ:
            pass
        elif cmd[hand.Command._KIND] == hand.Command._CMD_KIND_STOP:
            self.env.hand.stop()
        elif cmd[hand.Command._KIND] == hand.Command._CMD_KIND_CHANGE:
            self.env.hand.set_mode(cmd[hand.Command._MODE])
        else: #hand.Command._CMD_KIND_MOVE
            if cmd[hand.Command._FINGER] == hand.Finger.All:
                self.env.hand.move(cmd[hand.Command._POSITION], cmd[hand.Command._SPEED], cmd[hand.Command._FORCE])
            else:
                self.env.hand.move_finger(cmd[hand.Command._FINGER], cmd[hand.Command._POSITION], cmd[hand.Command._SPEED], cmd[hand.Command._FORCE]) 


        # prepare the state
        self.env.update() # note that the sim update is called twice, once here and once by the arm's sim_connection
        self.env.hand.read()
        state[hand.State._TIME] = self.env.time()
        state[hand.State._FLAGS] = hand.State._FLAG_READY \
                            + hand.State._FLAG_MOVING * self.env.hand.is_moving() \
                            + hand.State._FLAG_OBJECT_DETECTED * self.env.hand.object_detected()
        
        state[hand.State._MODE] = self.env.hand.mode()
        positions = self.env.hand.positions()
        state[hand.State._POSITION_A] = positions[0]
        state[hand.State._POSITION_B] = positions[1]
        state[hand.State._POSITION_C] = positions[2]
        targets = self.env.hand.targets()
        state[hand.State._TARGET_A] = targets[0]
        state[hand.State._TARGET_B] = targets[1]
        state[hand.State._TARGET_C] = targets[2]

        return state

 