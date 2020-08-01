# Roman - User Guide

The robot.py module contains the interface ([Robot](robot.py) class) for controlling a Universal Robots UR5 arm equipped with a Robotiq Force/Torque Sensor and a Robotiq 3-Finger Gripper. A Robot instace can be connected to a real robot or a simulation (pybullet)
The Robot class holds instances of the [Arm](ur/arm.py) and [Hand](rq/hand.py) classes, which can be accessed via the Robot.arm and Robot.hand fields. The Robot class provides additional functionality that spans the hand, arm and force sensor. 

## Dependencies
Roman requires numpy, scipy and pybullet. A real robot can come in handy too, but is not required. 

## Example

  ```python
  # import this package
  from roman import *
  
  # start up a robot simulation
  robot = connect(use_sim=True)
  
  # arm poses can be expresses in tool space (carthesian coordinates xyzrpy) 
  p1 = arm.Tool(-0.4, -0.4, 0.2, 0, math.pi, 0)
  # ... or joint angles
  p2 = arm.Joints(0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0)

  # move the arm to an abolute pose and block until the pose is reached
  robot.arm.move(p1)

  # most methods provide a non-blocking version
  robot.arm.move(p2, blocking=False)

  # close the gripper while the arm is moving, and wait for it to close
  robot.arm.close()

  # see whether the arm finished moving
  print(robot.arm.state.is_done())

  ```

For more examples, check out the examples directory.

## Further reading:
[Arm](ur/readme.md)
[Hand](rq/readme.md)
[Sim](sim/readme.md)

