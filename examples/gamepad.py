try:
    import inputs
except:
    print('This sample needs the inputs package (pip install inputs).')
    exit()

from roman import *
import math
DPAD_UP = 1
DPAD_DOWN = 2
DPAD_LEFT = 4
DPAD_RIGHT = 8
BTN_MENU = 16
BTN_BACK = 32
THUMB_STICK_PRESS_LEFT = 64
THUMB_STICK_PRESS_RIGHT = 128
BTN_SHOULDER_LEFT = 256
BTN_SHOULDER_RIGHT = 512
BTN_A = 4096
BTN_B = 8192
BTN_X = 16384
BTN_Y = 32768

def normalize_thumb_value(v):
    # eliminate deadzone and normalize
    return (v - 8000 * v/abs(v) if abs(v) > 8000 else 0) / (32768.0 - 8000)

def get_gamepad_state():
    return inputs.devices.gamepads[0]._GamePad__read_device().gamepad

if __name__ == '__main__':
    robot = connect() #connect(use_sim=False)
    done = False
    print('Use thumbsticks and dpad to move the arm.'
          'Use triggers to open/close gripper.'
          'Press A to change the grasp mode (pinch vs basic).'
          'Press B to switch between control modes (joint vs position).'
          'Press the back button to exit.')

    joint_control = True
    while not done:
        gps = get_gamepad_state()
        if gps.buttons == BTN_B:
            while gps.buttons == BTN_B:
                gps = get_gamepad_state()
            joint_control = not joint_control
            if joint_control:
                print("Using joint speed control.")
            else:
                print("Using end-effector position control.")
        if joint_control:
            # analog thumb sticks control the first four joints
            base = -normalize_thumb_value(gps.r_thumb_x)
            shoulder = normalize_thumb_value(gps.r_thumb_y)
            elbow = -normalize_thumb_value(gps.l_thumb_y)
            wrist1 = normalize_thumb_value(gps.l_thumb_x)
            # D-PAD controls wrists
            wrist2 = 1 if gps.buttons == DPAD_LEFT else -1 if gps.buttons == DPAD_RIGHT else 0
            wrist3 = 1 if gps.buttons == DPAD_UP else -1 if gps.buttons == DPAD_DOWN else 0
            target = JointSpeeds(base=base, shoulder=shoulder, elbow=elbow, wrist1=wrist1, wrist2=wrist2, wrist3=wrist3)
            robot.arm.speed(target, acc=1, blocking=False)
        else:
            # position control
             # analog thumb sticks control the end effector position plus wrist3 rotation
            x = 0.1 * normalize_thumb_value(gps.r_thumb_x)
            y = 0.1 * normalize_thumb_value(gps.r_thumb_y)
            z = 0.1 * -normalize_thumb_value(gps.l_thumb_y)

            yaw = normalize_thumb_value(gps.l_thumb_x)
            # D-PAD controls wrists
            roll = 1 if gps.buttons == DPAD_LEFT else -1 if gps.buttons == DPAD_RIGHT else 0
            pitch = 1 if gps.buttons == DPAD_UP else -1 if gps.buttons == DPAD_DOWN else 0
            target = robot.arm.state.tool_pose().to_xyzrpy() + [x, y, z, roll, pitch, yaw]
            target = Tool.from_xyzrpy(target)
            robot.arm.move(target,max_speed=1, max_acc=1, blocking=False)

        if gps.left_trigger > robot.hand.state.position():
            robot.hand.move(gps.left_trigger)
        elif gps.right_trigger > 255 - robot.hand.state.position():
            robot.hand.move(255 - gps.right_trigger)

        # A btn changes grasp 
        if gps.buttons == BTN_A:
            mode = GraspMode.PINCH if robot.hand.state.mode() != GraspMode.PINCH else GraspMode.BASIC
            robot.hand.set_mode(mode)

        # Back btn exits
        done = gps.buttons == BTN_BACK 

    robot.disconnect()
