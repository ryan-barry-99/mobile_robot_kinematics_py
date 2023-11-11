# Mobile Robot Kinematics

This program provides a Python implementation of the kinematics calculations for a fixed-wheeled mobile robot. It allows you to calculate the forward and inverse kinematics for any robot configuration.

## Features

- Calculates the linear and angular velocities of the robot given wheel velocities (forward kinematics).
- Calculates the required wheel velocities to achieve a desired robot velocity (inverse kinematics).

## Requirements

- Python 3.x
- numpy library (install using `pip install numpy`)

## Usage

1. Add wheels to the MobileRobot `__init__` function to define the robot's configuration.

2. Instantiate a MobileRobot object in your python application.

3. Use the `calculate_robot_velocity()` and `calculate_wheel_velocities()` functions to perform forward and inverse kinematic calculations of the robot.