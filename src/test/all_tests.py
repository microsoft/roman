import arm_unit_test
import arm_functional_test
import arm_sim_test
import arm_upload_test
import hand_functional_test
import manipulator_test

real_robot = True
if __name__ == '__main__':
    arm_unit_test.run()
    arm_functional_test.run(real_robot)
    arm_sim_test.run()
    arm_upload_test.run(real_robot)
    hand_functional_test.run(real_robot)
    manipulator_test.run(real_robot)

