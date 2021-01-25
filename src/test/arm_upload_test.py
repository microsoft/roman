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
from roman import ur
from roman.ur import loader

#############################################################
# These tests verify the UR script generation and uploading
#############################################################
def generate_script():
    print(f"Running {__file__}::{validate_script_syntax.__name__}()")
    script = ur.Connection()._Connection__generate_urscript()
    # outF = open("complete.script", "w")
    # outF.writelines(script)
    # outF.close()
    print(script)
    print("Passed.")

def validate_script_syntax():
    '''
    Uploads all the scripts (by loading test.script) to validate their syntax.
    If the test hangs, edit test.script and remove the imports one by one (from bottom to top)
    '''
    print(f"Running {__file__}::{validate_script_syntax.__name__}()")
    script_folder = os.path.join(os.path.join(os.path.join(rootdir, 'roman'), 'ur'), 'realtime')
    defs = [f"UR_CLIENT_IP=\"{ur.UR_DEFAULT_CLIENT_IP}\"", f"UR_CLIENT_PORT={ur.UR_DEFAULT_CLIENT_PORT}"]
    rt_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rt_socket.connect((ur.UR_ROBOT_IP, ur.UR_RT_PORT))
    print(f"loading test")
    script = loader.load_script(script_folder, "test", imports=[], defs = defs)
    loader.socket_send_retry(rt_socket, script.encode('ascii'))

    reverse_conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    reverse_conn.bind((ur.UR_DEFAULT_CLIENT_IP, ur.UR_DEFAULT_CLIENT_PORT))
    reverse_conn.listen(1)
    print('Waiting for robot to confirm... ')
    ctrl_socket, addr = reverse_conn.accept() 
    if (addr[0] != ur.UR_ROBOT_IP):
        raise RuntimeError("Invalid client connection")
    print('Script uploaded.')

    print(f"loading no_op")
    script = loader.load_script(script_folder, "no_op", defs = defs)
    loader.socket_send_retry(rt_socket, script.encode('ascii'))

    print('Script uploaded.')
    rt_socket.close()
    print("Passed.")

def test_script():
    print(f"Running {__file__}::{test_script.__name__}()")
    '''Use this to verify connectivity to the UR controller '''
    rt_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rt_socket.connect(('192.168.1.2', 30003))
    script = "textmsg(get_actual_tcp_pose())\n"
    loader.socket_send_retry(rt_socket, script.encode('ascii'))
    rt_socket.close()
    print("Passed.")

#############################################################
# Runner
#############################################################
def run(use_sim):
    # run the tests
    generate_script()

    if not use_sim:
        test_script()
        validate_script_syntax()
   
if __name__ == '__main__':
    run(False)
