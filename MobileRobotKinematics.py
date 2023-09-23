'''
This module provides the `MobileRobotKinematics` class which calculates the kinematic properties of any fixed-wheel mobile robot

Default values for the `MobileRobotKinematics` class are defined at the top of the file. Changing these values to fit any robot
configuration will update the kinematics accordingly.

Requirements: numpy

Author: Ryan Barry
Date created: September 14, 2023
'''

import numpy as np
from math import cos, sin, pi

WHEEL_RADIUS = 2*0.0254 # Wheel radius in meters
BASE_WIDTH = [8*.0254, 8*.0254, 8*.0254] # Distance from wheel to center of each wheel in meters
WHEEL_NAMES = ['wheel1', 'wheel2', 'wheel3'] # A list of names for the wheels
NUM_WHEELS = len(WHEEL_NAMES)
BETA = [0, 0, 0] # Angle between center of robot and wheel rotation axis for each wheel
GAMMA = [0, 0, 0] # 90 - angle between wheel rotation axis and roller rotation axis for each wheel
ALPHA = [pi/self.__num_wheels + 2*i*pi/self.__num_wheels for i in range(self.__num_wheels)] # Angle offset from x axis of each wheel around center of robot 


class MobileRobotKinematics :
    def __init__(self, beta=BETA, wheel_names=WHEEL_NAMES, wheel_radius=WHEEL_RADIUS, L=BASE_WIDTH, alpha=ALPHA):
        """
        Initializes the `MobileRobotKinematics` instance.
        :param beta: The angle between center of robot and wheel rotation axis for each wheel
        :param wheel_names: The names of wheels in the robot.
        :param wheel_radius: The radius of the wheel in meters.
        :param L: The distance from the center of the robot to the center of the wheels in meters.
        """

        # Define variables
        self.__beta = beta
        self.__wheel_radius = wheel_radius
        self.__L = L
        self.__wheel_names = wheel_names
        self.__num_wheels = len(self.__wheel_names)
        self.__alpha = alpha
        self.__r_theta = np.eye(self.__num_wheels)
        self.__J1_list = []
        self.__C1_list = []
        self.__J2 = np.eye(self.__num_wheels) * self.__wheel_radius
        for i, _ in enumerate(self.wheel_names):
            self.__J1_list.append(np.array([sin(self.__alpha[i] + self.__beta[i]), -cos(self.__alpha[i] + self.__beta[i]), -self.L[i]*cos(self.__beta[i])]))
            self.__C1_list.append(np.array([cos(self.__alpha[i] + self.__beta[i]), sin(self.__alpha[i] + self.__beta[i]), self.L[i]*sin(self.__beta[i])]))
        self.__J1 = np.array(self.__J1_list).T
        self.__C1 = np.array(self.__C1_list).T
        self.__zeta_dot = np.zeros(3,1)
        self.__phi = np.zeros(range(self.__num_wheels, 1))


    def calculate_robot_velocity(self, velocities):
        self.__phi = np.array(velocities)
        return self.forward_kinematics()
    
    def calculate_wheel_velocities(self, velocity):
        self.__zeta_dot = np.array(velocity)
        return self.inverse_kinematics()


    # Calculates the current linear and angular velocities of the robot
    def forward_kinematics(self):
        '''
        Calculates the vector of linear and angular velocities zeta_dot in the form 
                    [  x_dot  ] # linear velocity in the x direction
        zeta_dot =  [  y_dot  ] # linear velocity in the y direction
                    [ phi_dot ] # angular velocity about the z axis
        '''
        self.__zeta_dot = np.linalg.inv(self.__r_theta) @ self.__J1 @ self.__J2 @ self.__phi
        return self.__zeta_dot
  

    # Calculates the inverse kinematics of the robot
    def inverse_kinematics(self):
        '''
        Calculates the vector of wheel velocities necessary to achieve a targeted robot velocity
                [ wheel_1 ]
        phi =   [ wheel_2 ]
                [   ...   ]
                [ wheel_n ]
        '''
        self.__phi = np.linalg.inv(self.__J2) @ self.__J1 @ self.__r_theta @ self.__zeta_dot
        return self.__phi
    

    
