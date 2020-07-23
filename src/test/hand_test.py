import sys
import numpy as np
import math 
import time
import random
import os
import socket
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)
print(rootdir)
from roman import *
from roman.rq import *

def connection_test():
    print(f"Running {__file__}::{connection_test.__name__}()")
    state = State()
    con = Connection()    
    con.connect()
    cmd_close = Command.close()
    con.send(cmd_close, state)
    time.sleep(2)
    cmd_open = Command.open()
    con.send(cmd_open, state)
    time.sleep(2)
    con.disconnect()
    print("Passed.")

def controller_test():
    print(f"Running {__file__}::{connection_test.__name__}()")
    # check that a tight lop also works
    state = State()
    con = Connection()    
    con.connect()
    cmd_close = Command.close()
    con.send(cmd_close, state)
    while not state.is_done():
        con.send(cmd_close, state)
        time.sleep(1./125)
        cmd_close = Command.close()
    assert state.position_A() == Position.CLOSED

    cmd_open = Command.open()
    con.send(cmd_open, state)
    while not state.is_done():
        con.send(cmd_open, state)
        time.sleep(1./125)
    assert state.position_A() == Position.OPENED
    con.disconnect()
    print("Passed.")

def manipulator_test():
    print(f"Running {__file__}::{manipulator_test.__name__}()")
    
    m = connect(config={"real_robot":True})
    cmd = hand.Command.close()
    while not m.hand_state.is_done():
        m.cmd_hand(cmd)
    cmd = hand.Command.open()
    m.cmd_hand(cmd)
    while not m.hand_state.is_done():
        m.cmd_hand(cmd)
    assert m.hand_state.position_A() == Position.OPENED

    # cmd = hand.Command(Position.OPENED, mode=GraspMode.PINCH)
    # m.cmd_hand(cmd)
    # while not m.hand_state.is_done():
    #     m.cmd_hand(cmd)
    
    # cmd = hand.Command(Position.CLOSED, mode=GraspMode.PINCH)
    # m.cmd_hand(cmd)
    # while not m.hand_state.is_done():
    #     m.cmd_hand(cmd)

    m.disconnect()
    print("Passed.")

def run(real_robot = False):
    connection_test()
    if real_robot:
        controller_test()
        manipulator_test()
   
#env_test()
if __name__ == '__main__':
    run(True)