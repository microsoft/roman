import os
import numpy
import threading
import time
from robot.rq_gripper import Robotiq3FGripper, GraspMode, Finger
from robot.simenv import *
from robot.sim_connection import *
from robot.ur_connection import *
from robot.controllers import *
from robot.types import *
from robot.URScripts.constants import *
        
def server_loop(client, arm_config={}, hand_config={}, log_file=None):
    if log_file is not None:
        file = open(log_file, "wb")
        file.write(UR_PROTOCOL_VERSION)

    real_robot = arm_config.get("real_robot", 0) 
    # TODO: add arguments from config
    con = URConnection() if real_robot else SimURConnection()
    con.connect()
    arm = ArmController(con)
    # TODO: chain other controllers based on config 

    arm_cmd = Command()
    while True:
        cmd_is_new = client.poll()
        if cmd_is_new:
            client.recv_bytes_into(arm_cmd.array) # blocking
            #client.recv_bytes_into(hand_cmd) # blocking
        
        #rectify(arm_cmd, hand_cmd, arm_state, hand_state) # inject expected force, replace position delta with absolute, account for arm/hand/FT/tactile states
        arm_state = arm(arm_cmd)
        if cmd_is_new:
            client.send_bytes(arm_state.array)

        #log(file, arm_cmd, hand_cmd, arm_state, hand_state)
        if log_file is not None:
            file.write(arm_cmd, arm_state)

    # Disconnect the arm and gripper.
    con.disconnect()
