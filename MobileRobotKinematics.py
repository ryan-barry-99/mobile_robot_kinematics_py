'''
This module provides the `MobileRobotKinematics` class which calculates the kinematic properties of any fixed-wheel mobile robot

Default values for the `MobileRobotKinematics` class are defined at the top of the file. Changing these values to fit any robot
configuration will update the kinematics accordingly.

Requirements: numpy

Author: Ryan Barry
Date created: September 14, 2023
'''

import numpy as np
from math import cos, sin
from Wheel import Wheel


class MobileRobotKinematics :
    def __init__(self):
        """
        Initializes the `MobileRobotKinematics` instance.
        
        This class is used to calculate the kinematic properties of any fixed-wheel mobile robot.
        """

        # Define variables
        self.__wheels = []
        self.__num_wheels = 0
        self.update_jacobian()

    def add_wheel(self, name, radius, L, beta=0, gamma=0, alpha=0):
        wheel = Wheel(name, radius, L, beta, gamma, alpha)
        self.__wheels.append(wheel)
        self.__num_wheels += 1
        self.update_jacobian()
        return wheel
    
    def update_jacobian(self):
        self.__r_theta = np.eye(self.__num_wheels)
        self.__J1_list = []
        self.__C1_list = []
        self.__J2 = np.eye(self.__num_wheels)
        for i, wheel in enumerate(self.__wheels):
            self.__J2[i,i] = wheel.radius
            alpha = wheel.alpha
            beta = wheel.beta
            gamma = wheel.gamma
            L = wheel.L

            self.__J1_list.append(np.array([sin(alpha + beta + gamma), -cos(alpha + beta + gamma), -L*cos(beta + gamma)]))
            self.__C1_list.append(np.array([cos(alpha + beta + gamma), sin(alpha + beta + gamma), L*sin(beta + gamma)]))
            
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
        for i, wheel in enumerate(self.__wheels):
            wheel.set_velocity(self.__phi[i])
        return self.__phi
    

    
