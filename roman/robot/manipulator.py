import os
import numpy
import threading
import time
from roman.robot.rq_gripper import Robotiq3FGripper, GraspMode, Finger
from roman.robot.ur_arm import UR5Arm

class Manipulator(object):
    '''
    Unifies the arm, hand and sensors into a single entity that supports synchronous commands.
    Logs all arm/hand/sensor states while executing a command (including wait) if configured with a log file.
 
    '''
    def __init__(self, hand_ip='192.168.1.11'):
        self.arm = None
        self.hand = None

    def connect(self):
        '''Initializes the arm and gripper'''
        self.arm = UR5Arm()
        self.arm.connect()
        self.hand = Robotiq3FGripper()
        self.hand.connect()
        self.hand.close()
        self.sleep(0.5) # arm FT calibration

    def disconnect(self):
        '''Disconnects the arm and gripper.'''
        self.arm.stop(0.5)
        self.arm.disconnect()
        self.arm = None
        self.hand.disconnect()
        self.hand = None

    def sleep(self, duration, recalibrate = True):
        '''
        Sleeps for the specified duration (in seconds), optionally allowing the manipulator to recalibrate its FT sensor.
        Should be used instead of time.sleep() because it continues to log the manipulator state
        '''
        end = time.time() + duration
        if recalibrate:
            while time.time() < end:
                self.arm.recalibrate()
        else:
            while time.time() < end:
                self.arm.read()

    

def connect():
    m = Manipulator()
    m.connect()
    return m

