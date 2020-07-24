import numpy as np
import matplotlib.pyplot as plt
import time
import random
import math
import os
rootdir = os.path.dirname(os.path.dirname(__file__))
os.sys.path.insert(0, rootdir)
from roman import *

def plot_forces():
    m = connect(config={"real_robot":True})
    count = 2000
    x=range(count)
    ftfx=np.zeros(count)
    ftfy=np.zeros(count)
    ftfz=np.zeros(count)
    fttx=np.zeros(count)
    ftty=np.zeros(count)
    fttz=np.zeros(count)
    #fig, ax = plt.subplots(1)

    plt.style.use('ggplot')
    plt.ion()

    plt.subplot(3, 2, 1)
    plt.axis([0, count, -15, 15])
    l1 = plt.plot(x,ftfx, 'g:')
    plt.ylabel('Force X')

    plt.subplot(3, 2, 2)
    plt.axis([0, count, -2, 2])
    l2 = plt.plot(x,fttx, 'g:')
    plt.ylabel('Torque X')

    plt.subplot(3, 2, 3)
    plt.axis([0, count, -15, 15])
    l3 = plt.plot(x,ftfy, 'g:')
    plt.ylabel('Force Y')

    plt.subplot(3, 2, 4)
    plt.axis([0, count, -2, 2])
    l4 = plt.plot(x,ftty, 'g:')
    plt.ylabel('Torque Y')

    plt.subplot(3, 2, 5)
    plt.axis([0, count, -15, 15])
    l5 = plt.plot(x,ftfz, 'g:')
    plt.ylabel('Force Z')

    plt.subplot(3, 2, 6)
    plt.axis([0, count, -2, 2])
    l6 = plt.plot(x,fttz, 'g:')
    plt.ylabel('Torque Z')

    i=0
    start = time.time()
    poses = [
            ur.Command(target_position=ur.Joints(0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0), max_speed=1, max_acc=0.5), 
            ur.Command(target_position=ur.Joints(0, -math.pi/2, 0,0, math.pi/2, 0), max_speed=1, max_acc=0.5), 
            ur.Command(target_position=ur.Joints(0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0), max_speed=1, max_acc=0.5), 
            ur.Command(target_position=ur.Tool(-0.4, -0.4, 0.2, 0, math.pi, 0), max_speed=1, max_acc=0.5)
            ]
    try:
        ix_pose = 0
        while True:
            #pose = poses[random.randint(0,len(poses)-1)]
            pose = poses[ix_pose]
            ix_pose = (ix_pose+1)%len(poses)
            contact = False
            m.send_arm_cmd(pose)
            #while random.uniform(0,1) <0.98:
            while not m.arm_state.is_done():
                m.read_arm()
                # if m.arm_state.is_contact():
                #     break
                ft = m.arm_state.sensor_force()
                ftfx[i] = ft[0] 
                ftfy[i] = ft[1] 
                ftfz[i] = ft[2] 
                fttx[i] = ft[3] 
                ftty[i] = ft[4] 
                fttz[i] = ft[5] 
                i=(i+1)%count
                ftfx[i]=-90
                ftfy[i]=-90
                ftfz[i]=-90
                fttx[i]=-90
                ftty[i]=-90
                fttz[i]=-90

                if i%125 == 0:
                    l1[0].set_ydata(ftfx)
                    l2[0].set_ydata(fttx)
                    l3[0].set_ydata(ftfy)
                    l4[0].set_ydata(ftty)
                    l5[0].set_ydata(ftfz)
                    l6[0].set_ydata(fttz)

                    plt.pause(0.0001)
    except KeyboardInterrupt:
        pass
    m.disconnect()


if __name__ == '__main__':
    plot_forces()