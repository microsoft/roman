import os
import numpy
import threading
import time
from robot.rq_gripper import Robotiq3FGripper, GraspMode, Finger
from robot.arm import Arm
       
def server_loop(client_connection, log_file, arm_connection, environment): #, hand_connection):
    file = file_param
    if file is not None:
        file.write(UR_PROTOCOL_VERSION)

    #Initialize the arm and gripper
    arm_connection.connect()
    #hand.connect()

    # set up the controllers
    

    while True:
        cmd_is_new = client_con.poll()
        if cmd_is_new:
            client_con.recv_bytes_into(arm.state.array) # blocking
            client_con.recv_bytes_into(hand_cmd) # blocking
        
        rectify(arm_cmd, hand_cmd, arm_state, hand_state) # inject expected force, replace position delta with absolute, account for arm/hand/FT/tactile states
        arm.send(arm_cmd)
        hand.send(hand_cmd)
        arm.receive(arm_state)
        hand.receive(hand_state)
        log(file, arm_cmd, hand_cmd, arm_state, hand_state)

        # Make sure the arm response reflects the command.
        # The first arm response will likely not reflect it
        if cmd_is_new: 
            client_con.send(arm.state)
            client_con.send(hand.state)

    # Disconnect the arm and gripper.
    arm.stop(0.5)
    arm.disconnect()
    hand.disconnect()

