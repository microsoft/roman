import os
import numpy
import threading
import time
from multiprocessing import Process, Pipe, Event
from .rq import hand
from .ur import arm
from .server import server_loop

class Manipulator(object):
    '''
    Combines the manipulator components (arm, hand, FT and tactile sensors).
    '''
    _ARM_STOP_CMD = arm.Command()
    _ARM_READ_CMD = arm.Command.read()
    _HAND_STOP_CMD = hand.Command.stop()
    _HAND_READ_CMD = hand.Command.read()
    
    def __init__(self):
        self.arm_state = arm.State()
        self.hand_state = hand.State()

    def connect(self, config={}):
        self.__hand_server, hand_client = Pipe(duplex=True)
        self.__arm_server, arm_client = Pipe(duplex=True)
        self.__shutdown_event = Event()
        self.__process = Process(target=server_loop, args=(arm_client, hand_client, self.__shutdown_event, config))
        self.__process.start()

    def disconnect(self):
        self.__shutdown_event.set()
        self.__process.join()

    def cmd_arm(self, cmd):
        self.__arm_server.send_bytes(cmd.array)
        self.__arm_server.recv_bytes_into(self.arm_state.array)

    def stop_arm(self):
        self.cmd_arm(Manipulator._ARM_STOP_CMD)

    def read_arm(self):
        self.cmd_arm(Manipulator._ARM_READ_CMD)

    def cmd_hand(self, cmd):
        self.__hand_server.send_bytes(cmd.array)
        self.__hand_server.recv_bytes_into(self.hand_state.array)

    def stop_hand(self):
        self.cmd_hand(Manipulator._HAND_STOP_CMD)

    def read_hand(self):
        self.cmd_hand(Manipulator._HAND_READ_CMD)

    def move_simple(self, dx, dy, dz, dyaw, gripper_state):
        '''
        Moves the arm relative to the current position in carthesian coordinates, 
        assuming the gripper is vertical (aligned with the z-axis), pointing down.
        This supports the simplest Gym robotic manipulation environment.
        '''
        pass

def connect(config={}):
    m = Manipulator()
    m.connect(config)
    return m

