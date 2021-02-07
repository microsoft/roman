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

class GraspMode:
    CURRENT = -1
    BASIC = 0
    PINCH = 2
    WIDE = 4
    SCISSOR = 6

class Finger:
    A = 0
    B = 1
    C = 2
    All = 3

class Position:
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
    def target(self): return self[State._TARGET_A]
    def position(self): return self[State._POSITION_A]
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
    _CMD_KIND_CHANGE = 4

    def __init__(self):
        super().__init__(Command._BUFFER_SIZE, dtype=np.int16)
        self[Command._KIND] = Command._CMD_KIND_READ

    def make(self, kind, finger, position, speed, force, mode):
        self[Command._KIND] = kind
        self[Command._MODE] = mode 
        self[Command._FINGER] = finger
        self[Command._POSITION] = position
        self[Command._SPEED] = speed
        self[Command._FORCE] = force
        return self

    def kind(self): return self[Command._KIND]
    def mode(self): return self[Command._MODE]
    def finger(self): return self[Command._FINGER]
    def position(self): return self[Command._POSITION]
    def speed(self): return self[Command._SPEED]
    def force(self): return self[Command._FORCE]

class Hand:        
    _READ_CMD = Command()

    def __init__(self, controller):
        self.controller = controller
        self.command = Command()
        self.state = State() 

    def __execute(self, blocking):
        self.controller.execute(self.command, self.state)
        while blocking and not self.state.is_done():
            self.read()

    def read(self):
        self.controller.execute(Hand._READ_CMD, self.state)
        return self.state

    def move(self, position, finger = Finger.All, speed = 255, force = 0, blocking = True):
        self.command.make(Command._CMD_KIND_MOVE, finger, position, speed, force, GraspMode.CURRENT)
        self.__execute(blocking)

    def stop(self, blocking = True):
        self.command.make(Command._CMD_KIND_STOP, Finger.All, 0, 0, 0, GraspMode.CURRENT)
        self.__execute(blocking)

    def close(self, speed = 255, force = 0, blocking = True):
        self.command.make(Command._CMD_KIND_MOVE, Finger.All, Position.CLOSED, speed, force, blocking)
        self.__execute(blocking)

    def open(self, speed = 255, force = 0, blocking = True):
        self.command.make(Command._CMD_KIND_MOVE, Finger.All, Position.OPENED, speed, force, blocking)
        self.__execute(blocking)

    def set_mode(self, mode):
        self.command.make(Command._CMD_KIND_CHANGE, Finger.All, 0, 0, 0, mode)
        self.__execute(True)

