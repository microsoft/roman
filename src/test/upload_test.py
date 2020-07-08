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

from robot.ur_connection import *

def generate_script():
    script = URConnection()._URConnection__generate_urscript()
    # outF = open("complete.script", "w")
    # outF.writelines(script)
    # outF.close()
    print(script)

def validate_script_syntax():
    script_folder = os.path.join(os.path.join(rootdir, 'robot'), 'urscripts')
    #for script_name in ["no_op", "timer", "math", "sensor", "arm", "drive", "communication", "main"]:
    # Loading each script one by one doesn't seem to work.
    # Instead, we test communication.script, which loads all other (except main, which cannot be loaded without also executing it)
    # If the robot is stuck at "Waiting for robot to connect...", it is likely because of a syntax error in one of the scripts.
    # In that case, load the suspect scripts one by one (e.g. to validate that drive.script is ok, replace "communication" with "drive" below).
    for script_name in ["no_op", "communication"]: 
        rt_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rt_socket.connect(('192.168.1.2', 30003))
        print(f"loading {script_name}")
        script = robot.utils.load_script(script_folder, script_name, defs = ["COM_CLIENT_IP=\"192.168.1.10\"", "COM_CLIENT_PORT=50003"])
        robot.utils.socket_send_retry(rt_socket, script.encode('ascii'))
        print('Script uploaded.')
        rt_socket.close()

def test_script():
    rt_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rt_socket.connect(('192.168.1.2', 30003))
    script = "popup(get_actual_tcp_pose())\n"
    robot.utils.socket_send_retry(rt_socket, script.encode('ascii'))
    rt_socket.close()


def run():
    # run the tests
    generate_script()
    #validate_script_syntax()
   
#env_test()
if __name__ == '__main__':
    run()
