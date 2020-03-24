################################################################
## robot.py
## Works by loading a script onto the robot and communicating with it over sockets.
## The script implements interruptable motion primitives and sends back state info from the hand and F/T sensor.
## In this version, the arm state is part of the response and the RT interface is not used.
## This simplifies staying in sync with the controller (or detecting when we are out of sync) 
################################################################
import socket
import numpy as np
import timeit
import struct
import time

################################################################
## script loader
################################################################
def load_script(dir, scriptfile, is_include=False, imports=[]):
    """Loads a .script file line by line, replacing import statements with the correspondig file, assumed to be in the same folder"""
    script = ""
    with open(dir + "\\" + scriptfile) as lines:
        for line in lines:
            if is_include and (line.startswith("def") or line.startswith("end")): # strip the top-level "def" or "end" in included file
                #script += "#" + line 
                continue
            if line.strip().startswith("import"):
                line = line.strip()[7:]
                if line not in imports:
                    script += load_script(dir, line, True)
                    imports.append(line)
            else:
                script += line
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
