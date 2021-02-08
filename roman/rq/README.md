# Hand - Robotiq 3-Finger Gripper

This directory contains the interfaces for controlling a Robotiq 3-Finger Gripper via Modbus TCP.
The manual can be found [here](https://assets.robotiq.com/website-assets/support_documents/document/3-Finger_PDF_20190221.pdf)

The hand.py module contains the Hand class, which is the main entry point for controlling the gripper, together with the supporting State class and some relevant enums.

The Hand class can be connected, at initialization time, to a real gripper or to a simulation. Typically the initialization is done by the main Robot class, which holds an instance of Hand.

The contract between the Hand class and the underlying hardware and sim layers is fully contained in the State and Command classes. Rather than attempting to implement a Modbus server for the simulated hand, the code bifurcates at connection level (connection.py and sim_connection.py). The sim-specfic code (i.e. pybullet-specific) is further encapsulated in the ..\sim\rq3.py module, which should make it relatively easy to add support for a different simulator.

 

