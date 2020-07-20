import os
import numpy
import threading
import time
from multiprocessing import Process, Pipe
from .hand.rq_gripper import Robotiq3FGripper, GraspMode, Finger
from .arm.types import *
from .server import server_loop

class Manipulator(object):
    '''
    Combines the manipulator components (arm, hand, FT and tactile sensors).
    '''
    STOP_CMD = Command()
    READ_CMD = Command.make_read_cmd()
    
    def __init__(self):
        self.arm_state = State()

    def connect(self, arm_config={}, hand_config={}):
        self.__server, client = Pipe(duplex=True)
        self.__process = Process(target=server_loop, args=(client, arm_config, hand_config))
        self.__process.start()

    def disconnect(self):
        self.__process.terminate()

    def execute(self, cmd):
        self.__server.send_bytes(cmd.array)
        self.__server.recv_bytes_into(self.arm_state.array)

    def stop(self):
        self.execute(STOP_CMD)

    def read(self):
        self.execute(READ_CMD)

    def move_simple(self, dx, dy, dz, dyaw, gripper_state):
        '''
        Moves the arm relative to the current position in carthesian coordinates, 
        assuming the gripper is vertical (aligned with the z-axis), pointing down.
        This supports the simplest Gym robotic manipulation environment.
        '''
        pass

def connect():
    m = Manipulator()
    m.connect()
    return m

