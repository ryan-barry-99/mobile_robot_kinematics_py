"""
File: MobileRobot.py

Description: This file contains the implementation of the MobileRobot class, which represents a mobile robot with various functionalities.

Author: Ryan Barry
Date: November 11, 2023


Usage:
1. Instantiate the MobileRobot class.
2. Set up the robot's configuration by adding wheels.
3. Use the provided methods to control the robot's kinematics.

Notes:
- The class docstring contains an example implementation of a differential drive robot configuration.
- Make sure to update the wheel  parameters according to your specific robot configuration.

"""

from MobileRobotKinematics import MobileRobotKinematics

class MobileRobot(MobileRobotKinematics):
    def __init__(self):
        """
        Initializes the `MobileRobot` instance.
        
        This is where you will define the configuration of your robot by adding wheels.

        Example: Definition of a Differential Drive Robot
            Update the __init__ function as follows:
                self.leftWheel = self.add_wheel(name="leftWheel", radius=0.25, L=0.5, alpha=pi/2, beta=0, beta=0)
                self.rightWheel = self.add_wheel(name="rightWheel", radius=0.25, L=0.5, alpha=3*pi/2, beta=0, beta=0)

            Alternatively, you can define it through a class instance:
                diffDrive = MobileRobot()
                diffDrive.leftWheel = diffDrive.add_wheel(name="leftWheel", radius=0.25, L=0.5, alpha=pi/2, beta=0, beta=0)
                diffDrive.rightWheel = diffDrive.add_wheel(name="rightWheel", radius=0.25, L=0.5, alpha=3*pi/2, beta=0, beta=0)
        """
        super().__init__()

        


if __name__ == "__main__":
    robot = MobileRobot()