################################################################
## robot.py
## Works by loading a script onto the robot and communicating with it over sockets.
## The script implements interruptable motion primitives and sends back state info from the arm and F/T sensor.
## In this version, the arm state is part of the response and the RT interface is not used.
## This simplifies staying in sync with the controller (or detecting when we are out of sync) 
################################################################
import socket
import numpy as np
import timeit
import struct
import time
import math
from enum import Enum
import os
from ..common import *
from .loader import *
from .arm import *
from .realtime.constants import *

################################################################
## Real robot implementation
################################################################
class Connection:
    """Reads real robot state (arm and F/T sensor) and commands the robot in real-time."""
    def __init__(self, robot_ip=UR_ROBOT_IP, local_ip=UR_DEFAULT_CLIENT_IP, local_port=UR_DEFAULT_CLIENT_PORT):
        self.robot_ip = robot_ip
        self.local_ip = local_ip
        self.local_port = local_port
        self.__ctrl_socket = None
        self.__raw_state = bytearray(2048)
        self.__raw_cmd = bytearray(512)

    def __generate_urscript(self, name = "main"):
        constants = [f"UR_CLIENT_IP=\"{self.local_ip}\"", f"UR_CLIENT_PORT={self.local_port}"]
        script_folder = os.path.join(os.path.dirname(__file__), 'realtime')
        script = load_script(script_folder, name, defs=constants) 
        return script

    def connect(self):
        # create and connect the real-time channel
        rt_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rt_socket.connect((self.robot_ip, UR_RT_PORT))
        print('Connected to robot.')

        # now create the control channel and accept the connection from the script that will be running on the robot
        reverse_conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        reverse_conn.bind((self.local_ip, self.local_port))
        reverse_conn.listen(1)
        reverse_conn.settimeout(1)

        # test the version of the controller by running a script that attempts to connect back 
        script = self.__generate_urscript("version_test").encode('ascii')
        socket_send_retry(rt_socket, script)
        print('Checking version ... ')
        main = "main" # e-series entry-point
        try:
            self.__ctrl_socket, addr = reverse_conn.accept() 
            print('Controller is E-series')
            self.__ctrl_socket.close()
        except socket.timeout:
            main = "main_cb2" # CB2 entry point
            print('Controller is CB2')

        # upload the runtime, which will also connect back to us
        script = self.__generate_urscript(main).encode('ascii')
        socket_send_retry(rt_socket, script)
        print('Waiting for robot to connect... ')
        self.__ctrl_socket, addr = reverse_conn.accept() 
        if (addr[0] != self.robot_ip):
            raise RuntimeError("Invalid client connection")
        print('System ready.')

    def disconnect(self):
        if self.__ctrl_socket is not None:
            #__ctrl_socket.send("stop")
            self.__ctrl_socket.close()
            self.__ctrl_socket = None

        rt_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rt_socket.connect((self.robot_ip, UR_RT_PORT))
        script_folder = os.path.join(os.path.dirname(__file__), 'realtime')
        script = load_script(script_folder, "no_op")
        socket_send_retry(rt_socket, script.encode('ascii'))

    def execute(self, cmd, state):
        self.__send_cmd(cmd)
        return self.__receive_state(state)

    def __send_cmd(self, cmd):
        """Private. Encodes and sends the command to the robot. Must be followed by a call to __receive_state."""
        assert len(cmd) == UR_CMD_ENTRIES_COUNT
        i = 0
        self.__raw_cmd[0]=ord('(')
        
        for val in cmd.array:
            i = i + 1
            bytes = b'%f'%val
            length = len(bytes)
            self.__raw_cmd[i:i+length] = bytes
            i = i + length
            self.__raw_cmd[i] = 44 # ord(',') == 44

        self.__raw_cmd[i] =ord(')')
        length = i+1
        #print(self.__raw_cmd[:length])
        return socket_send_retry(self.__ctrl_socket, self.__raw_cmd, length)

    def __receive_state(self, state_buffer):
        """Private. Retrieves the response from the robot and parses it. Must be preceeded by a call to __send_cmd."""
        assert len(state_buffer) >= UR_STATE_ENTRIES_COUNT
        total = 0
        view = memoryview(self.__raw_state)
        while True:
            received = self.__ctrl_socket.recv_into(view)
            if received == 0:
                return False
            total = total + received
            if(view[received-1] == 93): #line ends with ']', ord(']')==93
                break
            view = view[received:]

        #Parse the response into a reusable state object
        i = 0
        start = 1 # skip starting [
        found = True
        view = memoryview(self.__raw_state)
        while found:
            end = self.__raw_state.find(44, start, total) # ord(',') == 44
            found = end > 0
            if not found: end = total - 1 #skip ending ]
            state_buffer[i] = float(view[start:end])
            i = i + 1
            start = end + 1
        return i==UR_STATE_ENTRIES_COUNT
