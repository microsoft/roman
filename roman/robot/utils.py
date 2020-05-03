################################################################
## utils.py
## Implements UR script loading and reliable socket operations.
################################################################
import os
import socket
import struct
import timeit
import time

import numpy as np

################################################################
## UR script loader
## The top-level script is wrapped in a function (UR requirement).
## Any definitions (e.g. constants) are added at the begining of main.
################################################################
def load_script(dir, module, is_include=False, imports=[], defs = []):
    """Loads a .script file line by line, replacing import statements with the correspondig file, assumed to be in the same folder"""
    script = f"def {module}():\n" if not is_include else ""
    for line in defs:
        script += "\t" + line + "\n"
         
    filename = os.path.join(dir, f"{module}.script")
    with open(filename) as lines:
        for line in lines:
            if line.strip().startswith("import"):
                line = line.strip()[7:]
                if line not in imports:
                    imports.append(line)
                    script += load_script(dir, line, True, imports)
            else:
                script += "\t" + line
    if not is_include:
        script += "\nend"
    return script

################################################################
## float as int encoding
################################################################
FP_FLOAT_FACTOR = 10000
def bytes2fpfloat(data, precision_factor = FP_FLOAT_FACTOR):
    val = int((data[0] << 24) + (data[1]<<16) + (data[2]<<8) + (data[3]))
    if (val & 0x80000000) > 0:
        val = -(0xFFFFFFFF - val)
    return val / precision_factor

def fpfloat2int(data, precision_factor = FP_FLOAT_FACTOR):
    return data * precision_factor

################################################################
## socket operations
################################################################
def socket_send_retry(socket, buf, size = 0):
    total = 0
    if size == 0: size = len(buf)
    while size > total:
        sent = socket.send(buf[total:size])
        if sent == 0:
            return False
        total = total + sent
    return True

def socket_receive_retry(socket, buf, size = 0):
    view = memoryview(buf)
    if size == 0: size = len(buf)
    while size > 0:
        received = socket.recv_into(view, size)
        if received == 0:
            return False
        view = view[received:]
        size = size - received
    
    return True
