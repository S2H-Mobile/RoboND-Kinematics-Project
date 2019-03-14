# RoboND-Kinematics-Project
This is a solution of the [Robotic arm - Pick & Place project](https://github.com/udacity/RoboND-Kinematics-Project) as part of the Robotics Nanodegree by Udacity. The inverse kinematics problem for the KUKA KR210 robotic arm is solved in order to perform a pick and place operation in a simulated Gazebo environment.

## Contents
- [IK_server.py](https://github.com/S2H-Mobile/RoboND-Kinematics-Project/blob/master/IK_server.py) is the Python module that solves the inverse kinematics problem.
- [IK_debug.py](https://github.com/S2H-Mobile/RoboND-Kinematics-Project/blob/master/IK_debug.py) is a Python script for testing the inverse kinematics solution in ``IK_server.py``.
- [kinematics_test.ipynb](https://github.com/S2H-Mobile/RoboND-Kinematics-Project/blob/master/kinematics_test.ipynb) is a Jupyter notebook for testing the forward and inverse kinematics.
-  The [writeup report](https://github.com/S2H-Mobile/RoboND-Kinematics-Project/blob/master/writeup/writeup_pick_and_place.md) describes the implemented solution.
- [writeup/kinematics.pdf](https://github.com/S2H-Mobile/RoboND-Kinematics-Project/blob/master/writeup/kinematics.pdf) provides additional background in geometry and further aspects of the KR210 arm design.


## Setup
This project is tested on the Jetson TX 2 with Ubuntu 16.04 LTS, Gazebo 7.0.0, ROS Kinetic.

1. Follow the setup instructions of the [Udacity Kinematics project](https://github.com/udacity/RoboND-Kinematics-Project).
2. Replace [IK_server.py](https://github.com/udacity/RoboND-Kinematics-Project/blob/master/kuka_arm/scripts/IK_server.py) by the one provided [here](https://github.com/S2H-Mobile/RoboND-Kinematics-Project/blob/master/IK_server.py).
3. Replace [IK_debug.py](https://github.com/udacity/RoboND-Kinematics-Project/blob/master/IK_debug.py) by the one provided [here](https://github.com/S2H-Mobile/RoboND-Kinematics-Project/blob/master/IK_debug.py).

## Usage
Follow the instructions of the [Udacity Kinematics project](https://github.com/udacity/RoboND-Kinematics-Project) to run the simulation.
