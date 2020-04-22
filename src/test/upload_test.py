import sys
import numpy as np
import math 
import time
import random
import os
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)
print(rootdir)

import robot

def generate_script():
    arm = robot.UR5Arm()
    script = arm._UR5Arm__generate_urscript()
    # outF = open("complete.script", "w")
    # outF.writelines(script)
    # outF.close()
    print(script)

def check_scripts():
    script_folder = os.path.join(os.path.join(rootdir, 'robot'), 'urscripts')
    for script_name in ["no_op", "timer"]: #"math", "sensor", "arm", "drive", "communication", "main"]:
        print(f"loading {script_name}")
        script = robot.utils.load_script(script_folder, script_name, defs = ["COM_CLIENT_IP=\"192.168.1.10\"", "COM_CLIENT_PORT=50003"])
        print(script)
        #utils.socket_send_retry(rt_socket, script.encode('ascii'))
        print('Script uploaded.')



check_scripts()

