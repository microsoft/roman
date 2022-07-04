try:
    import oculus_reader
    import pyquaternion
except:
    print('This sample needs the oculus_reader package (pip install git+https://github.com/microsoft-fevieira/oculus_reader.git#egg=oculus_reader). and (pip install pyquaternion)')
    exit()

from oculus_reader.reader import OculusReader
from roman import *
import roman.utils.transformation_utils as tr
import time
from pyquaternion import Quaternion
import numpy as np
from scipy.spatial.transform import Rotation

def oculus_to_robot(current_vr_transform):
    current_vr_transform = tr.RpToTrans(Quaternion(axis=[0, 0, 1], angle=-np.pi / 2).rotation_matrix, np.zeros(3)).dot(
        tr.RpToTrans(Quaternion(axis=[1, 0, 0], angle=np.pi / 2).rotation_matrix, np.zeros(3))).dot(current_vr_transform)

    return current_vr_transform

def get_pose_and_button(reader):
    poses, buttons = reader.get_transformations_and_buttons()

    if 'r' not in poses:
        return None, None, None, None

    return poses['r'], buttons['RTr'], buttons['rightTrig'][0], buttons['RG']

def get_cartesian_pose(robot, matrix=True):
    tool_pose = robot.arm.state.tool_pose()
    if not matrix:
        return tool_pose.position(), tool_pose.orientation()
    
    return tr.RpToTrans(tr.eulerAnglesToRotationMatrix(tool_pose[3:]), tool_pose[:3])

def set_gripper_position(robot, position):
    if position > 0:
        robot.hand.set_mode(GraspMode.PINCH)
    else:
        robot.hand.set_mode(GraspMode.BASIC)


if __name__ == '__main__':
    robot = connect(use_sim=True)
    reader = OculusReader()
    done = False
    reference_vr_transform = None
    current_vr_transform = None
    prev_handle_press = False
    initial_vr_offset = None

    joint_control = True
    while not done:
        time.sleep(1)
        current_vr_transform, trigger, trigger_continuous, handle_button = get_pose_and_button(reader)
        if current_vr_transform is None:
            continue
        else:
            if not prev_handle_press and handle_button:
                print("resetting reference pose")
                reference_vr_transform = oculus_to_robot(current_vr_transform)
                initial_vr_offset = tr.RpToTrans(np.eye(3), reference_vr_transform[:3, 3])
                reference_vr_transform = tr.TransInv(initial_vr_offset).dot(reference_vr_transform)

                reference_robot_transform = get_cartesian_pose(robot, matrix=True)

            if not handle_button:
                reference_vr_transform = None
                reference_robot_transform = get_cartesian_pose(robot, matrix=True)
                prev_handle_press = False
                continue

        prev_handle_press = True
        print('gripper set point', 1 - trigger_continuous)
        set_gripper_position(robot, 1 - trigger_continuous)

        current_vr_transform = oculus_to_robot(current_vr_transform)
        current_vr_transform = tr.TransInv(initial_vr_offset).dot(current_vr_transform)
        delta_vr_transform = current_vr_transform.dot(tr.TransInv(reference_vr_transform))

        M_rob, v_rob = tr.TransToRp(reference_robot_transform)
        M_delta, v_delta = tr.TransToRp(delta_vr_transform)
        new_robot_transform = tr.RpToTrans(M_delta.dot(M_rob), v_rob + v_delta)

        R, p = tr.TransToRp(new_robot_transform)
        euler = tr.rotationMatrixToEulerAngles(R)

        current_pose = get_cartesian_pose(robot, matrix=False)
        print(f"cur {current_pose}")

        target = Tool.from_xyzrpy(p + euler)
        print(f"target {target}")
        robot.arm.move(target, max_speed=1, max_acc=1, blocking=False)

    robot.disconnect()
