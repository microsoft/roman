import sys
import numpy as np
import math 
import time
import random
import os
import time
from multiprocessing import Process, Pipe
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)
from robot.types import *
from robot.manipulation_server import server_loop

def direct_read():
    print(f"Running {__file__}::{direct_read.__name__}()")

    server, client = Pipe(duplex=True)
    p = Process(target=server_loop, args=(client,))
    p.start()

    cmd = Command.make_read_cmd()
    state = State()
    server.send_bytes(cmd.array)
    server.recv_bytes_into(state.array)
    assert np.any(state.array)
    p.terminate()
    print("Passed.")   

def direct_move():
    print(f"Running {__file__}::{direct_move.__name__}()")

    server, client = Pipe(duplex=True)
    p = Process(target=server_loop, args=(client,))
    p.start()

    cmd = Command(target_position=Tool(-0.4, -0.4, 0.3,0, math.pi/2, math.pi))
    state = State()
    while not state.is_done():
        server.send_bytes(cmd.array)
        server.recv_bytes_into(state.array)
    p.terminate()
    print("Passed.")    
    
def run():
    direct_read()
    direct_move()
#env_test()
if __name__ == '__main__':
    run()
