import types_test
import controller_test
import sim_test
import server_test
import upload_test
import ur_test
import manipulator_test

real_robot = False
if __name__ == '__main__':
    types_test.run()
    sim_test.run()
    controller_test.run()
    server_test.run()
    ur_test.run(real_robot)
    upload_test.run(real_robot)
    manipulator_test.run()