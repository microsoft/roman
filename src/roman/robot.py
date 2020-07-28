import os
import numpy as np 
import threading
import time
from multiprocessing import Process, Pipe, Event
from .rq import hand
from .ur import arm
from .server import server_loop


class Robot(object):
    '''
    Combines the manipulator components (arm, hand, FT and tactile sensors).
    '''
    class PipeConnection(object):
        def __init__(self, pipe):
            self.pipe = pipe
        def execute(self, cmd, state):
            self.pipe.send_bytes(cmd.array)
            self.pipe.recv_bytes_into(state.array)

    def connect(self, config={}):
        __hand_server, hand_client = Pipe(duplex=True)
        __arm_server, arm_client = Pipe(duplex=True)
        self.__shutdown_event = Event()
        self.__process = Process(target=server_loop, args=(arm_client, hand_client, self.__shutdown_event, config))
        self.__process.start()

        self.arm = arm.Arm(Robot.PipeConnection(__arm_server))
        self.hand = hand.Hand(Robot.PipeConnection(__hand_server))

    def disconnect(self):
        self.__shutdown_event.set()
        self.__process.join()

    def move_simple(self, dx, dy, dz, dyaw, gripper_state):
        '''
        Moves the arm relative to the current position in carthesian coordinates, 
        assuming the gripper is vertical (aligned with the z-axis), pointing down.
        This supports the simplest Gym robotic manipulation environment.
        '''
        current = self.arm.state.tool_pose()
        current += [dx,dy, dz,0,0,0]
        self.arm.move(current)
        pass

def connect(config={}):
    m = Robot()
    m.connect(config)
    return m

