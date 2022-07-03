try:
    import oculus_reader
    import transformations
except:
    print('This sample needs the oculus_reader package (pip install git+https://github.com/microsoft-fevieira/oculus_reader.git#egg=oculus_reader). and (pip install transformations)')
    exit()

from oculus_reader.reader import OculusReader
from roman import *
from transformations import euler_from_matrix
import time


if __name__ == '__main__':
    robot = connect(use_sim=True)
    reader = OculusReader()
    done = False

    joint_control = True
    while not done:
        time.sleep(1)
        transform, buttons = reader.get_transformations_and_buttons()
        if not transform:
            continue

        right_controller_pose = transform['r']

        translation = right_controller_pose[:3, 3]
        x = translation[0]
        y = translation[1]
        z = translation[2]
        # print(x, y, z)

        rotation = euler_from_matrix(right_controller_pose)
        roll = rotation[0]
        pitch = rotation[1]
        yaw = rotation[2]
        # print(roll, pitch, yaw)

        current_pose = robot.arm.state.tool_pose().to_xyzrpy()
        print(f"cur {current_pose}")
        target = [x, y, z, roll, pitch, yaw]
        target = Tool.from_xyzrpy(target)
        print(f"target xyzrpy {target}")
        print(f"target {target}")
        robot.arm.move(target, max_speed=1, max_acc=1, blocking=False)

    robot.disconnect()
