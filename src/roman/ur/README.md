# Arm - Universal Robots UR5

This directory contains the interfaces for controlling a Universal Robots UR5 arm via a network connection.
The supported version is 1.8 (CB2 series), and the UR manual can be found [here](https://s3-eu-west-1.amazonaws.com/ur-support-site/17224/scriptmanual_en.pdf).

The arm.py module contains the Arm class, which is the main entry point for controlling the arm, together with the supporting State class and some relevant enums. The Arm class can be connected, at initialization time, to a real arm or to a simulation. Typically the initialization is done by the main Robot class, which holds an instance of Arm.

The implementation contains a real-time layer (inside the realtime directory) which combines force sensing with arm control. This layer runs on the UR controller (when using the real robot) or inside the Python interpreter (when using the simultion). It is written in a mix of Python and URScript, with the goal of keeping as much as possible in Python, so that most of it is shared between the real-robot and the simulated robot stacks. The Python code is limited to a subset of language compatible with URScript, with the addition of a few macros to fill in the gaps. When running on the real robot, a script loader parses the Python files and replaces the macros before loading the code to the UR controller.

The higher-level controllers are outside of the realtime directory, and are implemented in Python. They run on the "client" side (i.e. not on the UR controller), but need a tight execution loop, ideally running at the same frequency as the UR controller (125Hz for CB2, 500Hz for higher versions). When using the Robot class, this part of the code runs in a separate process.

TheThe sim-specfic code (i.e. pybullet-specific) is encapsulated in the ..\sim\ur.py module, which should make it relatively easy to add support for a different simulator.
 

