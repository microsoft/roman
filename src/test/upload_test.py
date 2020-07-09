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
import robot.utils
from robot.ur_connection import *
from robot.URScripts.constants import UR_RT_PORT, UR_ROBOT_IP, UR_DEFAULT_CLIENT_IP, UR_DEFAULT_CLIENT_PORT

def generate_script():
    script = URConnection()._URConnection__generate_urscript()
    # outF = open("complete.script", "w")
    # outF.writelines(script)
    # outF.close()
    print(script)

def validate_script_syntax():
    '''
    Uploads all the scripts (by loading test.script) to validate their syntax.
    If the test hangs, edit test.script and remove the imports one by one (from bottom to top)
    '''
    script_folder = os.path.join(os.path.join(rootdir, 'robot'), 'urscripts')
    defs = [f"UR_CLIENT_IP=\"{UR_DEFAULT_CLIENT_IP}\"", f"UR_CLIENT_PORT={UR_DEFAULT_CLIENT_PORT}"]
    rt_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rt_socket.connect((UR_ROBOT_IP, UR_RT_PORT))
    print(f"loading test")
    script = robot.utils.load_script(script_folder, "test", defs = defs)
    robot.utils.socket_send_retry(rt_socket, script.encode('ascii'))

    reverse_conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    reverse_conn.bind((UR_DEFAULT_CLIENT_IP, UR_DEFAULT_CLIENT_PORT))
    reverse_conn.listen(1)
    print('Waiting for robot to confirm... ')
    ctrl_socket, addr = reverse_conn.accept() 
    if (addr[0] != UR_ROBOT_IP):
        raise RuntimeError("Invalid client connection")
    print('Script uploaded.')

    print(f"loading no_op")
    script = robot.utils.load_script(script_folder, "no_op", defs = defs)
    robot.utils.socket_send_retry(rt_socket, script.encode('ascii'))

    print('Script uploaded.')
    rt_socket.close()

def test_script():
    '''Use this to verify connectivity to the UR controller '''
    rt_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rt_socket.connect(('192.168.1.2', 30003))
    script = "textmsg(get_actual_tcp_pose())\n"
    robot.utils.socket_send_retry(rt_socket, script.encode('ascii'))
    rt_socket.close()


def run():
    # run the tests
    generate_script()
    test_script()
    validate_script_syntax()
   
#env_test()
if __name__ == '__main__':
    run()
