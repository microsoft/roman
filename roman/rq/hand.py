################################################################
## hand.py
## Communicates with Robotiq 3-finger gripper over MODBUS TCP
################################################################
import socket
import numpy as np
import time
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
    NARROW = 8  # this is for sim support. On real hardware this is just PINCH.  
    

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
    _CMD_ID = 1
    _FLAGS = 2
    _MODE = 3
    _TARGET_A = 4
    _POSITION_A = 5
    _CURRENT_A = 6
    _TARGET_B = 7
    _POSITION_B = 8
    _CURRENT_B = 9
    _TARGET_C = 10
    _POSITION_C = 11
    _CURRENT_C = 12
    _TARGET_S = 13
    _POSITION_S = 14
    _CURRENT_S = 15
    _RESERVED = 16
    _BUFFER_SIZE = 17

    _FLAG_READY = 1
    _FLAG_INCONSISTENT = 2
    _FLAG_FAULTED = 4
    _FLAG_MOVING = 8
    _FLAG_OBJECT_DETECTED = 32

    def __init__(self):
        super().__init__(State._BUFFER_SIZE, dtype=np.int32)

    def time(self): return self[State._TIME] / 1000
    def cmd_id(self): return self[State._CMD_ID]
    def is_ready(self): return self[State._FLAGS] & State._FLAG_READY != 0
    def is_inconsistent(self): return self[State._FLAGS] & State._FLAG_INCONSISTENT != 0
    def is_faulted(self): return self[State._FLAGS] & State._FLAG_FAULTED != 0
    def is_moving(self): return self[State._FLAGS] & State._FLAG_MOVING != 0
    def is_done(self): return self.is_ready() and not self.is_moving()
    def object_detected(self): return self[State._FLAGS] & State._FLAG_OBJECT_DETECTED != 0
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
    _ID = 0
    _TIME = 1
    _KIND = 2
    _MODE = 3
    _FINGER = 4
    _POSITION = 5
    _SPEED = 6
    _FORCE = 7
    _BUFFER_SIZE = 8

    _CMD_KIND_READ = 0
    _CMD_KIND_STOP = 1
    _CMD_KIND_MOVE = 2
    _CMD_KIND_CHANGE = 4

    def __init__(self, id=0):
        super().__init__(Command._BUFFER_SIZE, dtype=np.int32)
        self[Command._KIND] = Command._CMD_KIND_READ
        self[Command._ID] = id

    def make(self, kind, finger, position, speed, force, mode):
        self[Command._KIND] = kind
        self[Command._MODE] = mode
        self[Command._FINGER] = finger
        self[Command._POSITION] = position
        self[Command._SPEED] = speed
        self[Command._FORCE] = force
        return self

    def id(self): return self[Command._ID]
    def time(self): return self[Command._TIME]
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
        self.last_cmd_id = 0

    def __execute(self, blocking):
        self.last_cmd_id += 1
        self.command[Command._ID] = self.last_cmd_id
        self.command[Command._TIME] = int(time.perf_counter()*1000) / 1000 # ms rounding, for consistency with the arm
        self.controller.execute(self.command, self.state)
        while blocking and(self.state.cmd_id() != self.command.id() or not self.state.is_done()):
            self.step()

    def execute(self, command, blocking):
        self.command[:] = command
        self.__execute(blocking)

    def step(self):
        last_time = self.state.time()
        while last_time == self.state.time() or self.state.cmd_id() != self.command.id():
            self.controller.execute(self.command, self.state)

    def read(self):
        self.execute(Hand._READ_CMD, self.state)

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

    def normalize(position):
        return position / 255

