import arm_unit_test
import arm_functional_test
import arm_sim_test
import arm_upload_test
import hand_functional_test
import manipulator_test

use_sim = True
if __name__ == '__main__':
    arm_unit_test.run()
    arm_functional_test.run(use_sim)
    arm_sim_test.run()
    arm_upload_test.run(use_sim)
    hand_functional_test.run(use_sim)
    manipulator_test.run(use_sim)
    print("All passed.")

