try:
    import inputs
except:
    print('This sample needs the inputs package (pip install inputs).')
    exit()

from roman import *
import math

def normalize_thumb_value(v):
    # eliminate deadzone and normalize
    return (v - 8000 * v/abs(v) if abs(v) > 8000 else 0) / (32768.0 - 8000)

def get_gamepad_state():
    return inputs.devices.gamepads[0]._GamePad__read_device().gamepad

if __name__ == '__main__':
    robot = connect(use_sim=True)
    done = False
    print("Use thumbsticks to move the arm.")
    print("Use triggers to open/close gripper.")
    print("Press the back button to exit.")

    while not done:
        gps = get_gamepad_state()
        d0 = 0.1 * normalize_thumb_value(gps.r_thumb_x)
        d1 = 0.1 * normalize_thumb_value(gps.r_thumb_y)
        d2 = - (0.1 * normalize_thumb_value(gps.r_thumb_y) + 0.1 * normalize_thumb_value(gps.l_thumb_y))
        d5 = 0.1 * normalize_thumb_value(gps.l_thumb_x)
        pose = robot.arm.state.joint_positions()
        target = pose + [d0, d1, d2, 0, 0, d5]
        target[3] = -target[2] - target[1] - math.radians(90)
        robot.arm.move(target, max_speed=3, max_acc=1, blocking=False)

        if gps.left_trigger > robot.hand.state.position():
            robot.hand.move(gps.left_trigger)
        elif gps.right_trigger > 255 - robot.hand.state.position():
            robot.hand.move(255 - gps.right_trigger)

        if gps.buttons == 4096:
            mode = GraspMode.PINCH if robot.hand.state.mode() != GraspMode.PINCH else GraspMode.BASIC
            robot.hand.set_mode(mode)

        done = gps.buttons == 0x20 # back btn exits

    robot.disconnect()
