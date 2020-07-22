import os
import numpy
import threading
import time
from . import rq
from . import ur
from .sim.ur import SimEnv
        
def server_loop(client, arm_config={}, hand_config={}, log_file=None):
    '''
    Control loop running at the same frequency as the hardware (e.g. 125Hz).
    It enables high-speed closed-loop control using force and tactile sensing (but no vision).
    '''
    if log_file is not None:
        file = open(log_file, "wb")
        #file.write(ur.UR_PROTOCOL_VERSION)

    real_robot = arm_config.get("real_robot", 0) 
    # TODO: add arguments from config
    env = None
    if real_robot:
        con = ur.Connection() 
    else:
        env = SimEnv()
        env.reset()
        con = ur.SimConnection(env)

    con.connect()
    arm_ctrl = ur.ArmController(con)
    # TODO: chain other controllers based on config 

    arm_cmd = ur.Command()
    while True:
        cmd_is_new = client.poll()
        if cmd_is_new:
            client.recv_bytes_into(arm_cmd.array) # blocking
            #client.recv_bytes_into(hand_cmd) # blocking
        
        #rectify(arm_cmd, hand_cmd, arm_state, hand_state) # replace position delta with absolute, account for arm/hand/FT/tactile states
        arm_state = arm_ctrl(arm_cmd)
        if cmd_is_new:
            client.send_bytes(arm_state.array)

        #log(file, arm_cmd, hand_cmd, arm_state, hand_state)
        if log_file is not None:
            file.write(arm_cmd, arm_state)

    # Disconnect the arm and gripper.
    con.disconnect()
    if env:
        env.disconnect()
