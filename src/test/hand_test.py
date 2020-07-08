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
import robot

def basic_com_test():
    hand = robot.Robotiq3FGripper()
    hand.connect()
    hand.open()
    hand.close()
    hand.disconnect()

   
def run():
    basic_com_test()
   
#env_test()
if __name__ == '__main__':
    run()