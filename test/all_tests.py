import arm_unit_test
import arm_functional_test
import arm_sim_test
import arm_upload_test
import hand_functional_test
import manipulator_test
import sim_test
import robot_replay_test

use_sim = True
if __name__ == '__main__':
    arm_unit_test.run()
    arm_functional_test.run(use_sim=use_sim)
    arm_sim_test.run()
    arm_upload_test.run(use_sim=use_sim)
    hand_functional_test.run(use_sim=use_sim)
    manipulator_test.run(use_sim=use_sim)
    sim_test.run()
    robot_replay_test.run(use_sim=use_sim)
    print("All passed.")

