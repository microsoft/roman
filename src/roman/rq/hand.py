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
    _TARGET_A = 3
    _POSITION_A = 4
    _CURRENT_A = 5
    _TARGET_B = 6
    _POSITION_B = 7
    _CURRENT_B = 8
    _TARGET_C = 9
    _POSITION_C = 10
    _CURRENT_C = 11
    _TARGET_S = 12
    _POSITION_S = 13
    _CURRENT_S = 14
    _RESERVED = 15
    _BUFFER_SIZE = 16
    
    _FLAG_READY = 1
    _FLAG_INCONSISTENT = 2
    _FLAG_FAULTED = 4
    _FLAG_MOVING = 8
    _FLAG_OBJECT_DETECTED = 32

    def __init__(self):
        super().__init__(State._BUFFER_SIZE, dtype=np.int16)

    def time(self): return self[State._TIME]
    def is_ready(self): return self[State._FLAGS] & State._FLAG_READY
    def is_inconsistent(self): return self[State._FLAGS] & State._FLAG_INCONSISTENT
    def is_faulted(self): return self[State._FLAGS] & State._FLAG_FAULTED
    def is_moving(self): return self[State._FLAGS] & State._FLAG_MOVING
    def is_done(self): return self.is_ready() and not self.is_moving()
    def object_detected(self): return self[State._FLAGS] & State._FLAG_OBJECT_DETECTED  
    def mode(self): return self[State._MODE]
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
    _KIND = 0
    _MODE = 1
    _FINGER = 2
    _POSITION = 3
    _SPEED = 4
    _FORCE = 5
    _BUFFER_SIZE = 6

    _CMD_KIND_READ = 0
    _CMD_KIND_STOP = 1
    _CMD_KIND_MOVE = 2

    def __init__(self, position, speed = 255, force = 0, mode=GraspMode.CURRENT, finger = Finger.All):
        super().__init__(Command._BUFFER_SIZE, dtype=np.int16)
        self[Command._KIND] = Command._CMD_KIND_MOVE 
        self[Command._MODE] = mode 
        self[Command._FINGER] = Finger.All
        self[Command._POSITION] = position
        self[Command._SPEED] = speed
        self[Command._FORCE] = force

    @staticmethod
    def stop():
        cmd = Command(Position.CURRENT)
        cmd[Command._KIND] = Command._CMD_KIND_STOP
        return cmd

    @staticmethod
    def read():
        cmd = Command(Position.CURRENT)
        cmd[Command._KIND] = Command._CMD_KIND_READ
        return cmd

    @staticmethod
    def close(speed = 255, force = 0, mode = GraspMode.CURRENT):
        return Command(position=Position.CLOSED, speed = speed, force = force, mode=mode, finger = Finger.All)

    @staticmethod
    def open(speed = 255, force = 0, mode = GraspMode.CURRENT):
        return Command(position=Position.OPENED, speed = speed, force = force, mode=mode, finger = Finger.All)

    @staticmethod
    def change(mode):
        return Command(position=Position.CURRENT, speed = 0, force = 0, mode=mode, finger = Finger.All)

