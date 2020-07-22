################################################################
## hand.py
## Communicates with Robotiq 3-finger gripper over MODBUS TCP
################################################################
import socket
import numpy as np
import timeit
import struct
import time
from ..common import *
from enum import Enum
import random

class GraspMode(object):
    CURRENT = -1
    BASIC = 0
    PINCH = 2
    WIDE = 4
    SCISSOR = 6

class Finger(object):
    All = 0
    A = 3
    B = 6
    C = 9

class Position(object):
    CURRENT = -1
    OPENED = 0
    CLOSED = 255

class State(Vec):
    '''Controls the Robotiq 3-finger gripper'''
    _TIME = 0
    _FLAGS = 1
    _MODE = 2
    _TARGET = 3
    _POSITION = 4
    _TARGET_A = 5
    _POSITION_A = 6
    _CURRENT_A = 7 
    _TARGET_B = 8
    _POSITION_B = 9
    _CURRENT_B = 10
    _TARGET_C = 11
    _POSITION_C = 12
    _CURRENT_C = 13
    _BUFFER_SIZE = 14
    
    _FLAG_READY = 1
    _FLAG_INCONSISTENT = 2
    _FLAG_FAULTED = 4
    _FLAG_MOVING = 8
    _FLAG_DONE = 16
    _FLAG_OBJECT_DETECTED = 32

    def __init__(self):
        super().__init__(State._BUFFER_SIZE)

    def time(self): return self[State._TIME]
    def is_ready(self): return self[State._FLAGS] & State._FLAG_READY
    def is_inconsistent(self): return self[State._FLAGS] & State._FLAG_INCONSISTENT
    def is_faulted(self): return self[State._FLAGS] & State._FLAG_FAULTED
    def is_moving(self): return self[State._FLAGS] & State._FLAG_MOVING
    def is_done(self): return self[State._FLAGS] & State._FLAG_DONE
    def object_detected(self): return self[State._FLAGS] & State._FLAG_OBJECT_DETECTED  
    def mode(self): return self[State._MODE]
    def target(self): return self[State._TARGET]
    def position(self): return self[State._POSITION]
    def target_A(self): return self[State._TARGET_A]
    def position_A(self): return self[State._POSITION_A]
    def current_A(self): return self[State._CURRENT_A]
    def target_B(self): return self[State._TARGET_B]
    def position_B(self): return self[State._POSITION_B]
    def current_B(self): return self[State._CURRENT_B]
    def target_C(self): return self[State._TARGET_C]
    def position_C(self): return self[State._POSITION_C]
    def current_C(self): return self[State._CURRENT_C]
    def grasp_size(self): return (128 - self.position_A()) + (128 - min(self.position_B(), self.position_C()))

class Command(Vec):
    '''Controls the Robotiq 3-finger gripper.'''
    _MODE = 0
    _FINGER = 1
    _POSITION = 2
    _SPEED = 3
    _FORCE = 4
    _BUFFER_SIZE = 5

    def __init__(self, position, speed = 255, force = 0, mode=GraspMode.CURRENT, finger = Finger.All):
        super().__init__(Command._BUFFER_SIZE, dtype=np.int8 )
        self[_MODE] = mode 
        self[_FINGER] = Finger.All
        self[_POSITION] = position
        self[_SPEED] = speed
        self[_FORCE] = force

    @staticmethod
    def stop():
        return Command(position=Position.CURRENT, speed = 0, force = 0, mode=GraspMode.CURRENT, finger = Finger.All)

    @staticmethod
    def close(speed = 255, force = 0, mode = GraspMode.CURRENT):

    @staticmethod
    def open(speed = 255, force = 0, mode = GraspMode.CURRENT):
        return Command(position=Position.OPENED, speed = speed, force = force, mode=mode, finger = Finger.All)

    @staticmethod
    def change(mode):
        return Command(position=Position.CURRENT, speed = 0, force = 0, mode=mode, finger = Finger.All)

