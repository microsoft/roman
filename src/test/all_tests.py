import arm_unit_tests
import arm_functional_test
import arm_sim_tests
import arm_upload_tests
import hand_functional_tests
import manipulator_tests

real_robot = True
if __name__ == '__main__':
    arm_unit_tests.run()
    arm_functional_test.run(real_robot)
    arm_sim_tests.run()
    arm_upload_tests.run(real_robot)
    hand_functional_tests.run(real_robot)
    manipulator_tests.run(real_robot)

