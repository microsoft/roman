
import time
from roman import connect_sim


def touch():
    '''This requires a horizontal surface that the arm can touch.'''
    print(f"Running {__file__}::{touch.__name__}()")
    (robot, scene) = connect_sim()
    home_pose = robot.arm.state.tool_pose().clone()
    below_table = home_pose.clone()
    below_table[2] = -0.2 # lower than the table
    #time.sleep(0.5)
    robot.arm.touch(below_table)
    assert robot.arm.state.is_goal_reached()

    # go back
    robot.arm.move(target_position=home_pose, force_low_bound=None, force_high_bound=None)

    robot.disconnect()
    print("Passed.")

def setup_sim(simenv):
    simenv.make_table()
    simenv.make_box([0.1, 0.1, 0.1], [-0.5, -0.1, 0.05], color=(0.8, 0.2, 0.2, 1), mass=0.1)

def pick():
    print(f"Running {__file__}::{pick.__name__}()")
    (robot, scene) = connect_sim(scene_init_fn=setup_sim)
    robot.hand.close()
    assert not robot.hand.state.object_detected()
    robot.hand.open()
    assert not robot.hand.state.object_detected()
    home_pose = robot.arm.state.tool_pose().clone()
    grasp_pose = home_pose.clone()
    grasp_pose[2] = 0.05
    robot.arm.move(target_position=grasp_pose, max_speed=2, max_acc=1, force_low_bound=None, force_high_bound=None)
    robot.hand.close()
    assert robot.hand.state.object_detected()
    robot.arm.move(target_position=home_pose, max_speed=2, max_acc=1, force_low_bound=None, force_high_bound=None)
    robot.hand.close()
    assert robot.hand.state.object_detected()
    robot.hand.open()
    time.sleep(0.5)
    robot.hand.close()
    assert not robot.hand.state.object_detected()

    robot.disconnect()
    print("Passed.")

#############################################################
# Runner
#############################################################
def run():
    touch()
    pick()


if __name__ == '__main__':
    run()

