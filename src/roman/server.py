import os
import numpy
import threading
import time
from . import rq
from . import ur
from .sim.ur_rq3 import SimEnv
        
def server_loop(arm_client, hand_client, shutdown_event, config={}, log_file=None, freq = 1./125):
    '''
    Control loop running at the same frequency as the hardware (e.g. 125Hz).
    It enables high-speed closed-loop control using force and tactile sensing (but no vision).
    '''
    if log_file is not None:
        file = open(log_file, "wb")
        #file.write(ur.UR_PROTOCOL_VERSION)

    real_robot = config.get("real_robot", 0) 

    # TODO: add arguments from config
    env = None
    if real_robot:
        arm_con = ur.Connection() 
        hand_con = rq.Connection()
    else:
        env = SimEnv()
        env.reset()
        arm_con = ur.SimConnection(env)
        hand_con = rq.SimConnection(env)

    arm_con.connect()
    hand_con.connect()
    arm_ctrl = ur.EMAForceCalibrator(ur.ArmController(arm_con))
    hand_ctrl = rq.HandController(hand_con)

    arm_cmd = ur.Command()
    hand_cmd = rq.Command.stop()
    while not shutdown_event.is_set():
        start_time = time.time()
        arm_cmd_is_new = arm_client.poll()
        if arm_cmd_is_new:
            arm_client.recv_bytes_into(arm_cmd.array) # blocking
            arm_cmd[ur.Command._ID] = time.time()
        
        hand_cmd_is_new = hand_client.poll()
        if hand_cmd_is_new:
            hand_client.recv_bytes_into(hand_cmd.array) # blocking
        
        #rectify(arm_cmd, hand_cmd, arm_state, hand_state) # replace position delta with absolute, account for arm/hand/FT/tactile states
        hand_state = hand_ctrl(hand_cmd)
        arm_state = arm_ctrl(arm_cmd)

        if arm_cmd_is_new:
            arm_client.send_bytes(arm_state.array)

        if hand_cmd_is_new:
            hand_client.send_bytes(hand_state.array)

        #log(file, arm_cmd, hand_cmd, arm_state, hand_state)
        if log_file is not None:
            file.write(arm_cmd, hand_cmd, arm_state, hand_state)

        if time.time()-start_time > 2*freq:
            print("Server loop lagging: " + str(time.time()-start_time))

    # Disconnect the arm and gripper.
    arm_con.disconnect()
    hand_con.disconnect()
    if env:
        env.disconnect()
