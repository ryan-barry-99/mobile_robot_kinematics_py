"""
File: wheel.py

TDescription: his file contains the implementation of the Wheel class, which represents a wheel in a mobile robot.

Author: Ryan Barry
Date Created: November 10, 2023
"""

class Wheel:
    def __init__(self, name, radius, L, alpha=0, beta=0, gamma=0):
        """
        Initializes a wheel object with the following parameters:
        :param name: The name of the wheel.
        :param radius: The radius of the wheel in meters.
        :param L: The distance from the center of the robot to the center of the wheel in meters.
        :param alpha: The angle offset from x axis of the wheel around center of robot.
        :param beta: The angle between center of robot and wheel's rotation axis.
        :param gamma: The angle between the wheel's roller rotation axis and the wheel's center (0 if no rollers).
        """
        self.name = name
        self.radius = radius
        self.L = L
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.velocity = 0.0
        self.target_velocity = 0.0

    def set_velocity(self, velocity):
        self.target_velocity = velocity