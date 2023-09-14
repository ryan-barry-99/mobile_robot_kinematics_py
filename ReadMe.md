# Mobile Robot Kinematics

This program provides a Python implementation of the kinematics calculations for a fixed-wheeled mobile robot. It allows you to calculate the forward and inverse kinematics for any robot configuration.

## Features

- Calculates the linear and angular velocities of the robot given wheel velocities (forward kinematics).
- Calculates the required wheel velocities to achieve a desired robot velocity (inverse kinematics).

## Requirements

- Python 3.x
- numpy library (install using `pip install numpy`)

## Usage

1. Update the wheel parameters in the code to match your robot's configuration. Modify the values of `WHEEL_RADIUS`, `BASE_WIDTH`, `WHEEL_NAMES`, `BETA`, and `GAMMA` at the top of the code.

2. Import the `MobileRobotKinematics` class from the `MobileRobotKinematics.py` module into your own Python script.

3. Instantiate the `MobileRobotKinematics` class with the desired wheel configuration: (robot = MobileRobotKinematics())

5. Use the `calculate_robot_velocity` method to calculate the linear and angular velocities of the robot given wheel velocities:

   robot_velocities = robot.calculate_robot_velocity([v1, v2, v3]))

    Replace `[v1, v2, v3]` with the actual wheel velocities.

7. Use the `calculate_wheel_velocities` method to calculate the required wheel velocities to achieve a desired robot velocity:
    python 
        wheel_velocities = robot.calculate_wheel_velocities([x_dot, y_dot, phi_dot])

    Replace `[x_dot, y_dot, phi_dot]` with the desired robot velocity.
