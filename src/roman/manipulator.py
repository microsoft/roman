import os
import numpy as np 
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
        print("Manipulator ready.")

    def disconnect(self):
        self.__shutdown_event.set()
        self.__process.join()

    def send_arm_cmd(self, cmd):
        self.__arm_server.send_bytes(cmd.array)
        self.__arm_server.recv_bytes_into(self.arm_state.array)

    def move_arm(self, cmd):
        self.send_arm_cmd(cmd)
        while not self.arm_state.is_done():
            self.read_arm()

    def stop_arm(self):
        self.move_arm(Manipulator._ARM_STOP_CMD)

    def read_arm(self):
        self.send_arm_cmd(Manipulator._ARM_READ_CMD)

    def send_hand_cmd(self, cmd):
        self.__hand_server.send_bytes(cmd.array)
        self.__hand_server.recv_bytes_into(self.hand_state.array)

    def move_hand(self, cmd):
        self.send_hand_cmd(cmd)
        while not self.hand_state.is_done():
            self.read_hand()

    def stop_hand(self):
        self.move_hand(Manipulator._HAND_STOP_CMD)

    def read_hand(self):
        self.send_hand_cmd(Manipulator._HAND_READ_CMD)

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

