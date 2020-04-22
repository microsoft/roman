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
from robot import utils
import math
from enum import Enum
import os

# known poses
class KnownPose(object):
    READY =[-0.4, -0.4, 0.3,0, math.pi, 0]
    PICK =[-0.4, -0.4, 0.10, 0, math.pi, 0]
    LOW_PICK = [-0.4, -0.4, 0, 0, math.pi, 0]
    SAFE =[-0.15, -0.15, 0.3,0, math.pi, 0]
    CLEAR = [0.1, -0.2, 0.3, 0, math.pi, 0]
    JOINT_HIDDEN = [math.pi, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]
    HIDDEN = [ 4.87232e-01,  1.07000e-01,  3.11646e-01,  2.21524e+00, -2.22699e+00, -8.07782e-04]

################################################################
## robot communication
################################################################

class UR5Arm(object):
    """Implements functionality to read robot state (arm, hand, F/T sensor) and command the robot in real-time."""

    # socket config
    LOCAL_PORT = 50003  # this is the port on the client side the robot will connect back to
    DEFAULT_ROBOT_IP = '192.168.1.2'
    DEFAULT_LOCAL_IP = '192.168.1.10'

    # robot settings
    DEFAULT_MASS = 3.25
    DEFAULT_TOOL_COG = [0, 0, 0.12]
    DEFAULT_TCP =  [0, 0, 0.12, 0, 0, 0]
    DEFAULT_MAX_SPEED = 2
    DEFAULT_ACCELERATION = 0.5
    DEFAULT_MAX_FORCE =  [25, 25, 25, 2, 2, 2] 
    NO_FORCE_LIMIT =  [1000, 1000, 1000, 1000, 1000, 1000]

    __RT_PORT = 30003 # real-time UR interface (RT)
    __STATE_ENTRIES_COUNT = 78 # this is how many numbers we expect to receive with every response
    __CMD_ENTRIES_COUNT = 22 # Total must stay under 30. 
    __VERSION = 1.0

    # messages
    __CMD_STOP = 0
    __CMD_READ = 1
    __CMD_MOVEJ = 2
    __CMD_MOVEP = 3
    __CMD_MOVE = 4
    __CMD_CONFIG = 10
    __CMD_CALIB = 11
   
    def __init__(self, robot_ip=DEFAULT_ROBOT_IP, local_ip=DEFAULT_LOCAL_IP, local_port = LOCAL_PORT, file = None):
        self.robot_ip = robot_ip
        self.local_ip = local_ip
        self.local_port = local_port
        self.__ctrl_socket = None
        self.__raw_state = bytearray(2048)
        self.__state = np.zeros(self.__STATE_ENTRIES_COUNT)
        self.__raw_cmd = bytearray(512)
        self.__cmd = np.zeros(self.__CMD_ENTRIES_COUNT)
        self.__file = file
        if file is not None:
            file.write(self.__VERSION)
            
    def __enter__(self): 
        self.connect()
        return self

    def __exit__(self,exc_type, exc_val, exc_tb):
        self.disconnect()

    def __generate_urscript(self):
        constants = [f"COM_CLIENT_IP=\"{self.local_ip}\"", f"COM_CLIENT_PORT={self.local_port}"]
        script_folder = os.path.join(os.path.dirname(__file__), 'urscripts')
        script = utils.load_script(script_folder, "main", defs=constants) 
        return script

    def connect(self):
        # create and connect the real-time channel
        rt_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rt_socket.connect((self.robot_ip, self.__RT_PORT))
        print('Connected to robot.')
            
        # upload our script, which will attempt to connect back to us
        script = self.generate_urscript().encode('ascii')
        utils.socket_send_retry(rt_socket, script)
        print('Script uploaded.')

        # now create the control channel and accept the connection from the script now running on the robot
        reverse_conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        reverse_conn.bind((self.local_ip, self.local_port))
        reverse_conn.listen(1)
        print('Waiting for robot to connect... ')
        self.__ctrl_socket, addr = reverse_conn.accept() 
        if (addr[0] != self.robot_ip):
            raise RuntimeError("Invalid client connection")
        # start the time counter (on Windows)
        time.clock()
        self.read()
        print('System ready.')

    def disconnect(self):
        if self.__ctrl_socket is not None:
            #__ctrl_socket.send("stop")
            self.__ctrl_socket.close()
            self.__ctrl_socket = None

        rt_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rt_socket.connect((self.robot_ip, self.__RT_PORT))
        script = utils.load_script("robot\\urscripts", "no_op.script")
        utils.socket_send_retry(rt_socket, script.encode('ascii'))

    def __update(self):
        """
        Sends the constructed command and reads the state of the robot arm and F/T sensor from our custom script running on the UR controller.
        The method blocks until all data is read. 
        """
        self.__send_cmd()
        self.__receive_state()
        
        if self.__file is not None:
            self.__file.write(timeit.default_timer())
            self.__file.write(self.__raw_cmd)
            self.__file.write(self.__raw_state)

    def __send_cmd(self):
        """Private. Encodes and sends the command to the robot. Must be followed by a call to __receive_state,
        cmd:
            __CMD_READ: cmd time, cmd code
            __CMD_STOP: cmd time, cmd code, acceleration
            __CMD_MOVE/__CMD_MOVEJ/__CMD_MOVEP: cmd time, cmd code, target pose (vec6), acc, max speed, tcp force limits (vec3), tcp torque limits (vec3), target speed (vec6)
            __CMD_CONFIG: cmd time, cmd code, payload (kg), tool center of gravity (vec3), tool tip (vec3)
            __CMD_CALIB: cmd time, cmd code
        """
        i = 0
        self.__raw_cmd[0]=ord('(')
        self.__cmd[0] = time.clock()
        
        for val in self.__cmd:
            i = i + 1
            bytes = b'%f'%val
            length = len(bytes)
            self.__raw_cmd[i:i+length] = bytes
            i = i + length
            self.__raw_cmd[i] = 44 # ord(',') == 44

        self.__raw_cmd[i] =ord(')')
        length = i+1
        #print(self.__raw_cmd[:length])
        return utils.socket_send_retry(self.__ctrl_socket, self.__raw_cmd, length)


    def __receive_state(self):
        """Private. Retrieves the response from the robot and parses it. Must be preceeded by a call to __send_cmd."""
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
            self.__state[i] = float(view[start:end])
            i = i + 1
            start = end + 1
        return i==self.__STATE_ENTRIES_COUNT


################################################################
## Robot state. 
################################################################
    def time(self): 
        '''The time when the response was generated, in robot time'''
        return self.__state[0]
    def joint_positions(self): 
        '''The current actual joint angular position vector in rad : [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]'''
        return self.__state[1:7]
    def joint_speeds(self): 
        '''The current actual joint angular velocity vector in rad/s: [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]'''
        return self.__state[7:13]
    def tool_pose(self): 
        '''The current actual TCP vector : ([X, Y, Z, Rx, Ry, Rz])'''
        return self.__state[13:19]
    def tool_speed(self): 
        '''The current actual TCP velocity vector; ([X, Y, Z, Rx, Ry, Rz])'''
        return self.__state[19:25]
    def target_joint_positions(self): 
        '''The current target joint angular position vector in rad: [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]'''
        return self.__state[25:31]
    def target_joint_speeds(self): 
        '''The current target joint angular velocity vector in rad/s: [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]'''
        return self.__state[31:37]
    def target_tool_pose(self): 
        '''The current target TCP vector; ([X, Y, Z, Rx, Ry, Rz])'''
        return self.__state[37:43]
    def target_tool_speed(self): 
        '''The TCP speed. The ﬁrst three values are the cartesian speeds along x,y,z, and the last three deﬁne the current rotation axis, rx,ry,rz, and the length|rz,ry,rz|deﬁnes the angular velocity in radians/s'''
        return self.__state[43:49]
    def tool_force(self):
       '''Returns the wrench (Force/Torque vector) at the TCP.
        The external wrench is computed based on the error between the joint torques required to stay on the trajectory and the expected joint torques. 
        The function returns ”p[Fx (N), Fy(N), Fz(N), TRx (Nm), TRy (Nm), TRz (Nm)]”,
        where Fx, Fy, and Fx are the forces in the axes of the robot base coordinate system measured in Newtons, and TRx, TRy, and TRz are the torques around these axes measyred in Newton times Meters. '''
       return self.__state[49:55]
    def joint_torques(self): 
        '''The torque on the joints, corrected by the torque needed to move the robot itself (gravity, friction, etc.), returned as a vector of length 6.'''
        return self.__state[55:61]
    def sensor_force(self): 
        '''The forces and moments reported by the Force/Torque sensor mounted on the wrist'''
        return self.__state[61:67]
    def tool_acceleration(self): 
        '''The accelerometer reading'''
        return self.__state[67:70] 
    def external_force(self): 
        '''The forces reported by the Force/Torque sensor mounted on the wrist, corrected by the accelerometer readings'''
        return self.__state[70:76] 
    def cmd_time(self): 
        '''The time of the last drive command, in robot time'''
        return self.__state[76]
    def cmd_state(self): 
        '''The status of the last command'''
        return self.__state[77]

################################################################
## commands
################################################################
    def async_move(self, pose, acc = DEFAULT_ACCELERATION, max_speed = DEFAULT_MAX_SPEED, max_force = DEFAULT_MAX_FORCE):
        '''Start moving to tool pose, linear in tool space. The function needs to be called in a tight loop, otherwise the motion stops as a security measure.'''
        self.__cmd[1] = self.__CMD_MOVE
        self.__cmd[2:8] = pose
        self.__cmd[8]=acc
        self.__cmd[9]=max_speed
        self.__cmd[10:16]=max_force
        self.__cmd[16:]=0
        self.__update()
        return self.cmd_state()

    def move(self, pose, acc = DEFAULT_ACCELERATION, max_speed = DEFAULT_MAX_SPEED, max_force = DEFAULT_MAX_FORCE, stop_condition = None):
        '''Moves to tool pose, linear in tool space. By default, stops when the pose is reached or an external force is detected.'''
        done = False
        while not done:
            self.async_move(pose, acc, max_speed, max_force)
            done =  self.cmd_state() == 0 if stop_condition is None else stop_condition(self)
        return self.cmd_state()

    def async_movej(self, position, acc = DEFAULT_ACCELERATION, max_speed = DEFAULT_MAX_SPEED, max_force = DEFAULT_MAX_FORCE):
        '''Start moving to joint position, linear in joint-space. The function needs to be called in a tight loop, otherwise the motion stops as a security measure.'''
        self.__cmd[1] = self.__CMD_MOVEJ
        self.__cmd[2:8] = position
        self.__cmd[8]=acc
        self.__cmd[9]=max_speed
        self.__cmd[10:16]=max_force
        self.__cmd[16:]=0
        self.__update()
        return self.cmd_state()

    def movej(self, position, acc = DEFAULT_ACCELERATION, max_speed = DEFAULT_MAX_SPEED, max_force = DEFAULT_MAX_FORCE, stop_condition = None):
        '''Moves to joint position, linear in joint-space. By default, stops when the pose is reached or an external force is detected.'''
        done = False
        while not done:
            self.async_movej(position, acc, max_speed, max_force)
            done =  self.cmd_state() == 0 if stop_condition is None else stop_condition(self)
        return self.cmd_state()

    def async_movep(self, pose, acc = DEFAULT_ACCELERATION, max_speed = DEFAULT_MAX_SPEED, max_force = DEFAULT_MAX_FORCE):
        '''Start moving to tool pose, linear in joint-space. The function needs to be called in a tight loop, otherwise the motion stops as a security measure.'''
        self.__cmd[1] = self.__CMD_MOVEP
        self.__cmd[2:8] = pose
        self.__cmd[8]=acc
        self.__cmd[9]=max_speed
        self.__cmd[10:16]=max_force
        self.__cmd[16:]=0
        self.__update()
        return self.cmd_state()

    def movep(self, pose, acc = DEFAULT_ACCELERATION, max_speed = DEFAULT_MAX_SPEED, max_force = DEFAULT_MAX_FORCE, stop_condition = None):
        '''Moves to tool pose, linear in joint-space. By default, stops when the pose is reached or an external force is detected.'''
        done = False
        while not done:
            self.async_movep(pose, acc, max_speed, max_force)
            done =  self.cmd_state() == 0 if stop_condition is None else stop_condition(self)
        return self.cmd_state()

    def read(self):
        '''Updates the robot state.'''
        self.__cmd[1] =self.__CMD_READ
        self.__cmd[2:]=0
        self.__update()
        
    def recalibrate(self):
        '''Allows the robot to recalibrate the force sensing.'''
        self.__cmd[1] =self.__CMD_CALIB
        self.__cmd[2:]=0
        self.__update()
    
    def async_stop(self, acc = 1):
        '''Stops the current motion.'''
        self.__cmd[1] =self.__CMD_STOP
        self.__cmd[2]=acc
        self.__cmd[3:]=0
        self.__update()

    def stop(self, acc = 1):
        '''Stops the current motion and waits for the motion to complete.'''
        while self.cmd_state() != 0:
            self.async_stop(acc)

    # needs work
    # The FT sensor is internally calibrated to account for the 3.2kg hand payload (using robotiq_FT_sensor_demo.exe or the UR script)
    # This function should be designed to be used when an object is grasped and the mass and tool position change
    def reset(self, tcp = DEFAULT_TCP, mass = DEFAULT_MASS, cog = DEFAULT_TOOL_COG):
        '''Reconfigures the robot.'''
        self.__cmd[1] = self.__CMD_CONFIG
        self.__cmd[2] = mass
        self.__cmd[3:6] = cog
        self.__cmd[6:12] = tcp
        self.__cmd[12:]= 0
        self.__update()
        raise Exception("NYI")

    def touch(self, pose, force = [3.0, 3.0, 3.0, 0.5, 0.5, 0.5]):
        ''' 
        Moves towards tool pose, linear in tool space. 
        Stops and returns true if the hand touched an obstacle, false if the arm reached the specified pose.
        '''
        contact_position = np.array(self.joint_positions())
        confirmation_count = 3
        count = confirmation_count
        while count > 0:
            contact_position[:] = self.joint_positions()
            res = self.move(pose, acc = 0.01, max_speed = 0.05, max_force = force, stop_condition = lambda r : r.cmd_state() <= 0)
            if res == 0: # stopped because we reached the goal with no external interference
                return False
            else: # stopped due to excess force, make sure this is not spurrious
                self.read()
                if np.all(np.abs(self.external_force()[0:6]) < force):
                    continue
                # did it stop in the same place? If yes count down, if not reset the counter
                if np.allclose(contact_position, self.joint_positions(), atol=0.01):
                    count = count - 1
                else:
                    count = confirmation_count
        return True
 