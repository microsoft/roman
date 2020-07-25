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

#############################################################
# These tests require a real hand for now (no sim option)
#############################################################
def connection_test(real_robot):
    print(f"Running {__file__}::{connection_test.__name__}()")
    state = State()
    con = Connection() if real_robot else SimConnection(None)
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
    print(f"Running {__file__}::{controller_test.__name__}()")
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

#############################################################
# Runner
#############################################################
def run(real_robot = False):
    connection_test(real_robot)
    if real_robot:
        controller_test()
   
if __name__ == '__main__':
    run(True)