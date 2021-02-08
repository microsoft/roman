import math 
import os
import random
import sys
import time
import socket

import numpy as np

rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)
print(rootdir)
from roman import rq
from roman.sim.simenv import SimEnv

#############################################################
# These tests require a real hand for now (no sim option)
#############################################################
def connection_test(con):
    print(f"Running {__file__}::{connection_test.__name__}()")
    hand = rq.hand.Hand(con)
    hand.close()
    assert hand.state.position() == rq.hand.Position.CLOSED
    hand.open()
    assert hand.state.position() == rq.hand.Position.OPENED
    print("Passed.")

def controller_test(con):
    print(f"Running {__file__}::{controller_test.__name__}()")
    # check that a tight loop also works
    hand = rq.hand.Hand(rq.controllers.HandController(con))
    
    assert hand.state.position() == rq.hand.Position.OPENED
    hand.close()
    assert hand.state.position() == rq.hand.Position.CLOSED

    hand.open()
    assert hand.state.position() == rq.hand.Position.OPENED
    print("Passed.")

def position_test(con):
    print(f"Running {__file__}::{position_test.__name__}()")
    hand = rq.hand.Hand(rq.controllers.HandController(con))
    for i in range(1, 16):
        hand.move(16*i-1)
        assert not hand.state.object_detected()
        hand.open()
        assert not hand.state.object_detected()

def mode_test(con):
    print(f"Running {__file__}::{mode_test.__name__}()")
    hand = rq.hand.Hand(rq.controllers.HandController(con))
    for i in [rq.hand.GraspMode.BASIC, rq.hand.GraspMode.PINCH, rq.hand.GraspMode.WIDE, rq.hand.GraspMode.SCISSOR]:
        hand.set_mode(i)
        hand.close()
        assert not hand.state.object_detected()
        assert hand.state.position() == rq.hand.Position.CLOSED
        hand.open()
        assert not hand.state.object_detected()
        assert hand.state.position() == rq.hand.Position.OPENED

#############################################################
# Runner
#############################################################
def run(use_sim = False):
    if not use_sim:
        con = rq.connection.Connection()
    else:
        env = SimEnv()
        env.connect()
        con = rq.sim_connection.SimConnection(env)
    con.connect()
    connection_test(con)
    controller_test(con)
    position_test(con)
    mode_test(con)
    con.disconnect()
    if use_sim:
        env.disconnect()
   
if __name__ == '__main__':
    run(True)