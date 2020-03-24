################################################################
## hand.py
## Communicates with Robotiq 3-finger gripper over MODBUS TCP
################################################################
import socket
import numpy as np
import timeit
import struct
import time
from robot import utils
from enum import Enum
import random

class GraspMode(object):
    BASIC = 0
    PINCH = 2
    WIDE = 4
    SCISSOR = 6

class Finger(object):
    A = 3
    B = 6
    C = 9

class Robotiq3FGripper(object):
    '''Controls the Robotiq 3-finger gripper'''
    
    def __init__(self, hand_ip='192.168.1.11'):
        self.__ip = hand_ip
        self.__write_buf = bytearray(29)
        self.__write_resp = bytearray(12)
        self.__write_registers = memoryview(self.__write_buf)[13:]
        self.__read_req = bytearray(12)
        self.__read_buf = bytearray(25)
        self.__read_registers = memoryview(self.__read_buf)[9:]
        self.__modbus_socket = None
        # https://en.wikipedia.org/wiki/Modbus
        struct.pack_into('>HHHBBHHB', self.__write_buf, 0, 
                         # start header
                         0,  # request ID (2 bytes) - we'll change this with every cmd
                         0,  # TCP (2 bytes)
                         23, # remaining size (2 bytes)
                         255,# slave address (1 bytes)
                         16, # op code (write multiple holding registers) (1 bytes)
                         # end header
                         # start body (for opcode 16)
                         0,  # address of first register
                         8,  # number of registers to write
                         16) # number of bytes to write
        
        struct.pack_into('>HHHBBHH', self.__read_req, 0, 
                         0, # request ID - we'll change this with every cmd
                         0,  # TCP
                         6, # remaining size
                         255,# slave address
                         4, # op code (read multiple input registers)
                         0,  # address of first register
                         8)  # number of registers to write


    def connect(self, activate = True):
        """Connects to the gripper"""        
        if self.__modbus_socket is not None:
            self.disconnect()
        self.__modbus_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__modbus_socket.connect((self.__ip, 502)) # 502 = MODBUS_PORT
        if activate:
            self.__write_registers[0] = 1
            self.__send_cmd() # activate
        #print('Connected to hand.')

    def disconnect(self):
        self.__modbus_socket.close()
        self.__modbus_socket = None

    def __send_cmd(self):
        """Sends a gripper command and returns without waiting for completion"""
        
        if self.__modbus_socket  is None:
            self.connect(False)

        # update the cmd id
        struct.pack_into('>H', self.__write_buf, 0, random.randint(0, 65536)) 

        #send the request
        try:
            utils.socket_send_retry(self.__modbus_socket, self.__write_buf)
        except Exception as e:
            self.connect(False)
            utils.socket_send_retry(self.__modbus_socket, self.__write_buf)
        
        #print("sent: ", self.__write_buf)
        
        # consume response
        # If this hangs, it's most likely because we got back a MODBUS error response, which is only 9 bytes
        utils.socket_receive_retry(self.__modbus_socket, self.__write_resp, 8+2+2) # header (8 bytes) + address of first written register (2 byte) + number of registers written (2 byte)

        # update state 
        self.read()
        

    def __print_registers(self, registers):
        for x in registers:
            print(hex(x), end=" ")
        print()

    def debug_dump(self):
        print("Write registers:")
        self.__print_registers(self.__write_registers)
        print()
        print("Read registers:")
        self.__print_registers(self.__read_registers)
        print()

    def read(self):
        """Reads the gripper state into a reusable state object."""

        if self.__modbus_socket  is None:
            self.connect(False)

         # update the cmd id
        struct.pack_into('>H', self.__read_req, 0, random.randint(0, 65535)) 

        # send the request
        try:
            utils.socket_send_retry(self.__modbus_socket, self.__read_req)
        except Exception as e:
            self.connect(False)
            utils.socket_send_retry(self.__modbus_socket, self.__read_req)

        # read the response
        # If this hangs, it's most likely because we got back a MODBUS error response, which is only 9 bytes
        utils.socket_receive_retry(self.__modbus_socket, self.__read_buf, 8+1+16)

    def is_inconsistent(self): 
        return ((self.__read_registers[0] & 0x07) != (self.__write_registers[0] & 0x07)) or self.__read_registers[Finger.A] != self.__write_registers[Finger.A] or self.__read_registers[Finger.B] != self.__write_registers[Finger.B] or self.__read_registers[Finger.C] != self.__write_registers[Finger.C]
    def is_ready(self): return self.__read_registers[0] & 0x31 == 0x31 and not self.is_inconsistent()
    def is_faulted(self): return self.__read_registers[2] != 0
    def is_moving(self): return self.__read_registers[0] & 0xC8 == 0x08  
    def object_detected(self): return self.is_ready() and self.__read_registers[1] != 0xFF
    def finger_status(self, finger): pass #TBD
    def mode(self): return self.__read_registers[0] & 0xF9
    def taget_position(self): return self.__read_registers[3]
    def position(self): return self.__read_registers[4]
    def finger_target_position(self, finger): return self.__read_registers[finger]
    def finger_position(self, finger): return self.__read_registers[finger+1]
    def finger_current(self, finger): return self.__read_registers[finger+2]
    def grasp_size(self): return (128 - self.__read_registers[Finger.A+1]) + (128 - min(self.__read_registers[Finger.B+1], self.__read_registers[Finger.C+1]))

    def deactivate(self): 
        self.__write_registers[0] = 0
        self.__send_cmd()

    # sets the mode to one of GraspMode values
    def set_mode(self, mode):
        self.__write_registers[0] = mode | 1 # include the activation bit, and move
        self.__send_cmd()

    def stop(self):
        self.__write_registers[0] = self.__write_registers[0] & 0xF7
        self.__send_cmd()

    def async_move(self, position, speed = 255, force = 0): 
        self.__write_registers[0] = self.__write_registers[0] | 8 # move
        self.__write_registers[1] = 0 # disable individual finger control
        self.__write_registers[Finger.A] = position
        self.__write_registers[Finger.A+1] = speed
        self.__write_registers[Finger.A+2] = force
        self.__write_registers[Finger.B] = position
        self.__write_registers[Finger.B+1] = speed
        self.__write_registers[Finger.B+2] = force
        self.__write_registers[Finger.C] = position
        self.__write_registers[Finger.C+1] = speed
        self.__write_registers[Finger.C+2] = force
        self.__send_cmd()

    def move(self, position, speed = 255, force = 0): 
        self.async_move(position, speed, force)
        self.ready_wait()

    def async_move_finger(self, finger, position, speed = 255, force = 0):
        self.__write_registers[0] = self.__write_registers[0] | 8 # move
        if (self.__write_registers[1] != 4):
            self.__write_registers[1] = 4
            # make sure the command reflects the current finger position
            self.__write_registers[Finger.B] = self.position()
            self.__write_registers[Finger.B+1] = 0
            self.__write_registers[Finger.B+2] = 0
            self.__write_registers[Finger.C] = self.position()
            self.__write_registers[Finger.C+1] = 0
            self.__write_registers[Finger.C+2] = 0
        
        self.__write_registers[finger] = position
        self.__write_registers[finger+1] = speed
        self.__write_registers[finger+2] = force
        self.__send_cmd()

    def move_finger(self, finger, position, speed = 255, force = 0):
        self.async_move_finger(finger, position, speed, force)
        self.ready_wait()

    def async_move_fingers(self, positionA, positionB, positionC, speedA = 255, speedB = 255, speedC = 255, forceA = 0, forceB = 0, forceC = 0):
        self.__write_registers[0] = self.__write_registers[0] | 8 # move
        self.__write_registers[1] = 4 # advanced mode (individual finger control)
        self.__write_registers[Finger.A] = positionA
        self.__write_registers[Finger.A+1] = speedA
        self.__write_registers[Finger.A+2] = forceA
        self.__write_registers[Finger.B] = positionB
        self.__write_registers[Finger.B+1] = speedB
        self.__write_registers[Finger.B+2] = forceB
        self.__write_registers[Finger.C] = positionC
        self.__write_registers[Finger.C+1] = speedC
        self.__write_registers[Finger.C+2] = forceC
        self.__send_cmd()

    def move_fingers(self, positionA, positionB, positionC, speedA = 255, speedB = 255, speedC = 255, forceA = 0, forceB = 0, forceC = 0):
        self.async_move_fingers(positionA, positionB, positionC, speedA, speedB, speedC, forceA, forceB, forceC)
        self.ready_wait()
        
    def close(self, speed = 255, force = 0):
        self.move(255, speed, force)

    def grasp(self, speed = 255, force = 0):
        self.set_mode(GraspMode.BASIC)
        self.close(speed, force)

    def pinch(self, speed = 255, force = 0):
        self.set_mode(GraspMode.PINCH)
        self.close(speed, force)

    def release(self, speed = 0):
        self.move(0, speed, 0)

    def open(self, speed = 255):
        self.move(0, speed, 0)

    def ready_wait(self):
        while not self.is_ready() or self.is_moving():
            time.sleep(0.01)
            self.read()



