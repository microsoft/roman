################################################################################
## Retrieves the arm state from the UR's built-in Real-Time interface. 
## Does not need a script running on the robot and can be used with the pendant.
## Note: This file is not used. It is kept for reference and debugging purposes only.
################################################################################
import socket
import numpy as np
import timeit
import struct
import time

RT_PORT = 30003 # real-time UR interface (RT)
RT_PACKET_SIZE = 812

class RealTimeInterface:
    """Implements functionality to read robot state."""
    def __init__(self, robot_ip, file = None):
        self.__rt_socket = None
        self.robot_ip = robot_ip
        self.__raw_rt_state = bytearray(RT_PACKET_SIZE) 
        self.__file = file

    def __enter__(self): 
        self.connect()
        return self

    def __exit__(self,exc_type, exc_val, exc_tb):
        self.disconnect()

    def connect(self):
        self.disconnect()

        # create and connect the real-time channel
        self.__rt_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__rt_socket.connect((self.robot_ip, RT_PORT))
        print('RT connected.')

    def disconnect(self):
         if self.__rt_socket:
            self.__rt_socket.close()

    def update(self):
        """
        Reads the state of the robot arm.
        The method blocks until all data is read. 
        """
        view = memoryview(self.__raw_rt_state)
        size = RT_PACKET_SIZE
        while size > 0:
            received = self.__rt_socket.recv_into(view, size)
            if received == 0:
                return False
            view = view[received:]
            size = size - received
        
        if (self.__file != None):
            self.__file.write(self.__raw_rt_state)

    #state access functions:
    def time(self): return struct.unpack_from("!d",self.__raw_rt_state, 4)[0]
    def q_target(self): return struct.unpack_from("!dddddd",self.__raw_rt_state, 4 + 8)
    def qd_target(self): return struct.unpack_from("!dddddd",self.__raw_rt_state, 4 + 7*8)
    def qdd_target(self): return struct.unpack_from("!dddddd",self.__raw_rt_state, 4 + 13*8)
    def i_target(self): return struct.unpack_from("!dddddd",self.__raw_rt_state, 4 + 19*8)
    def m_target(self): return struct.unpack_from("!dddddd",self.__raw_rt_state, 4 + 25*8)
    def q_actual(self): return struct.unpack_from("!dddddd",self.__raw_rt_state, 4 + 31*8)
    def qd_actual(self): return struct.unpack_from("!dddddd",self.__raw_rt_state, 4 + 37*8)
    def i_actual(self): return struct.unpack_from("!dddddd",self.__raw_rt_state, 4 + 43*8)
    def tool_acc_values(self): return struct.unpack_from("!ddd",self.__raw_rt_state, 4 + 49*8)
    def tcp_force(self): return struct.unpack_from("!dddddd",self.__raw_rt_state, 4 + 67*8) # 52*8 + 15*8 unused
    def tool_vector(self): return struct.unpack_from("!dddddd",self.__raw_rt_state, 4 + 73*8)
    def tcp_speed(self): return struct.unpack_from("!dddddd",self.__raw_rt_state, 4 + 79*8)
    def digital_input_bits(self): return struct.unpack_from("!Q",self.__raw_rt_state, 4 + 85*8)[0]
    def motor_temperature(self): return struct.unpack_from("!dddddd",self.__raw_rt_state, 4 + 86*8)
    def controller_timer(self): return struct.unpack_from("!Q",self.__raw_rt_state, 4 + 92*8)[0]
    def test_value(self): return struct.unpack_from("!d",self.__raw_rt_state, 4 + 93*8)[0]
    def robot_mode(self): return struct.unpack_from("!d",self.__raw_rt_state, 4 + 94*8)[0]
    def joint_modes(self): return struct.unpack_from("!dddddd",self.__raw_rt_state, 4 + 95*8)



with ur_rt_robot('192.168.1.2') as robo:
    while (True):
        robo.update()
        print("%2.4f %2.4f %2.4f"%robo.tool_acc_values(), end="\r")

    #robo.update(cmd)
    #print(robo.m_target())
    #print(robo.i_actual())
