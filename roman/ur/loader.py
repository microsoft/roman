################################################################
## utils.py
## Implements UR script loading and reliable socket operations.
################################################################
import socket
import numpy as np
import timeit
import struct
import time
import os
from ..common import *

################################################################
## UR script loader
## The top-level script is wrapped in a function (UR requirement).
## Any definitions (e.g. constants) are added at the begining of main.
################################################################
def load_script(dir, module, is_include=False, imports=[], defs = []):
    """
    Combines a script and its imports into a single in-memory string that can be loaded to the UR controller.
    The function looks for a file called module.script or module.py in the directory specified by dir.
    If the file is a .py , 'end' statements must still be present, prefixed with '#ur:', and import statements must be
    of the form 'from .foo import *'. The rest of the file needs to be fully compatible with the UR script language.
    Import statements are replaced with the correspondig file (.script or .py), assumed to be in the same folder.
    """
    script = f"def {module}():\n" if not is_include else ""
    for line in defs:
        script += "    " + line + "\n"
    
    filename = os.path.join(dir, f"{module}.script")
    if not os.path.exists(filename):
        filename = os.path.join(dir, f"{module}.py")

    with open(filename) as lines:
        for line in lines:
            if line.strip().startswith("from") or line.strip().startswith("import"):
                module = line.split()[1].split('.')[-1]
                if module not in imports:
                    imports.append(module)
                    script += load_script(dir, module, True, imports)
            elif line.strip().startswith("global"):
                pass
            elif line.strip().startswith("#ur:"):
                script += "    " + line.replace("#ur:", "")
            else:
                script += "    " + line
    if not is_include:
        script += "\nend\n"
    return script

################################################################
## debuging functions 
################################################################
def dump_pose():
    rt_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rt_socket.connect(('192.168.1.2', 30003))
    script = 'textmsg("TCP:", get_actual_tcp_pose())\n'
    robot.utils.socket_send_retry(rt_socket, script.encode('ascii'))
    script = 'textmsg("Joints:", get_actual_joint_positions())\n'
    robot.utils.socket_send_retry(rt_socket, script.encode('ascii'))
    rt_socket.close()