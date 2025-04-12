import math
from datetime import datetime

"""
This module parses the IMU data messages and computes the distance traveled and velocity between consecutive readings.
It maintains the state of the last received coordinates and timestamp.
"""


class IMUDataParser:
    """
    A parser for IMU data messages that computes the distance traveled and the velocity between consecutive readings.
    Maintains internal state for the previous coordinates and timestamp.
    """
    def __init__(self):
        # Initialize past values as None
        self.past_coordinates = None
        self.past_time = None

    def parse_imu_data(self, imu_message: str): 
        """
        Parses the IMU data message and calculates the distance traveled and velocity between the current and previous readings.
        
        Args:
            imu_message (str): The IMU data message containing coordinates in the format "X: value\tY: value\tZ: value".

        Returns:
            tuple: A tuple (distance, velocity) where:
                distance (float): The Euclidean distance between the previous and current coordinates.
                velocity (float): The computed velocity (distance per second).
                Returns (0.0, 0.0) if this is the first reading.
        """
        current_coordinates = self.parse_coordinates(imu_message)
        current_time = datetime.now()

        if self.past_coordinates is None or self.past_time is None:
            # No previous data available; initialize state and return zero values
            self.past_coordinates = current_coordinates
            self.past_time = current_time
            return (0.0, 0.0)

        # Calculate the Euclidean distance between the previous and current coordinates
        distance = self.calculate_distance(self.past_coordinates, current_coordinates)
        # Calculate the time difference in seconds
        time_diff = (current_time - self.past_time).total_seconds()
        # Calculate velocity as distance divided by time difference (avoid division by zero)
        velocity = distance / time_diff if time_diff > 0 else 0.0

        # Update the state for the next reading
        self.past_coordinates = current_coordinates
        self.past_time = current_time

        return (distance, velocity)

    @staticmethod
    def parse_coordinates(imu_coordinates_data: str):
        """
        Parses a tab-separated string of coordinates and returns a list of floats.
        
        Args:
            imu_coordinates_data (str): A string in the format "X: value\tY: value\tZ: value".
        
        Returns:
            list: A list of floats representing the coordinates [X, Y, Z].
        """
        return [float(part.split(": ")[1]) for part in imu_coordinates_data.split("\t")]

    @staticmethod
    def calculate_distance(past_coordinates, current_coordinates):
        """
        Calculates the Euclidean distance between two sets of coordinates in 3D space.
        
        Args:
            past_coordinates (list): A list of floats representing the first coordinate [X, Y, Z].
            current_coordinates (list): A list of floats representing the second coordinate [X, Y, Z].
        
        Returns:
            float: The Euclidean distance between the two coordinates.
        """
        return math.dist(past_coordinates, current_coordinates)
