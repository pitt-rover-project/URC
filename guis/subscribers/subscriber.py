#!/usr/bin/env python3
"""
Generic ROS2 Subscriber Class for Arduino Bridge Topics

This module contains a generic subscriber class that can be used to interface with any ROS2 topic.
It is intended to serve as a base class for various subscribers used in the GUI versions of the rover project.

Additionally, this file demonstrates how to subscribe to each Arduino bridge topic published by the ArduinoBridgeBase child classes:
    - MotorBridge publishes on the 'motor_data' topic
    - UltrasonicBridge publishes on the 'ultrasonic_data' topic
    - GPSBridge publishes on the 'gps_data' topic
    - IMUBridge publishes on the 'imu_data' topic

Each subscriber is instantiated with a unique node name and is spun concurrently using a MultiThreadedExecutor.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Import a standard message type for demonstration purposes. Replace with the appropriate message type as needed.
from std_msgs.msg import String
from guis.subscribers.IMUDataParser import IMUDataParser
from guis.subscribers.GPSDataParser import GPSDataParser
from PyQt5.QtCore import pyqtSignal, QObject

class GenericSubscriber(Node):
    """
    A generic ROS2 subscriber that subscribes to a given topic and processes incoming messages.

    This class extends the rclpy Node class and can be used as a base class for creating ROS2 topic subscribers.
    It subscribes to a specified topic and processes messages using a provided callback function.
    If no callback is provided, a default callback that logs the message is used.

    Attributes:
        topic_name (str): The name of the ROS2 topic to subscribe to.
        msg_type (Type): The ROS2 message type that is expected on the topic.
        callback (callable): The callback function that processes incoming messages.
    """

    def __init__(self, topic_name: str, msg_type, node_name: str = 'generic_subscriber', callback=None):
        """
        Initialize the GenericSubscriber.

        Args:
            topic_name (str): The name of the ROS2 topic to subscribe to.
            msg_type (Type): The message type of the ROS2 topic.
            node_name (str, optional): The name of the ROS2 node. Defaults to 'generic_subscriber'.
            callback (callable, optional): A function to be called when a message is received.
                If not provided, the default callback will be used.
        """
        # Initialize the ROS2 node with the provided unique name
        super().__init__(node_name)
        self.topic_name = topic_name
        self.msg_type = msg_type

        # Use the provided callback function or fall back to the default callback
        if callback is not None:
            print("Using provided callback function.")
            callback
        else:
            print("No callback function provided, using default callback.")
            callback = self.default_callback

        self.callback = callback if callback is not None else self.default_callback
        print(self.callback)
        # Create a subscription to the topic with a queue size of 10
        self.subscription = self.create_subscription(
            self.msg_type,     # Message type
            self.topic_name,   # Topic name
            self.callback,     # Callback function
            10                 # Queue size
        )
        print("------------ subscription created ------------")
        print(self.subscription)
        self.get_logger().info(f"Subscribed to topic: {self.topic_name}")


    def default_callback(self, msg):
        """
        Default callback function for processing incoming messages.

        This function logs the received message to the console.

        Args:
            msg: The message received from the subscribed topic.
        """
        print("TEST")
        self.get_logger().info(f"Received message on {self.topic_name}: {msg}")


class IMUSubscriber(GenericSubscriber, QObject):
    imu_data_updated = pyqtSignal(float, float, float)  # velocity, vert_tilt, horiz_tilt
    """
    A ROS2 subscriber that parses IMU messages using the IMUDataParser.

    This class overrides the default callback of the GenericSubscriber to parse
    the incoming IMU message and compute the distance traveled and velocity.
    """
    def __init__(self, topic_name: str = 'imu_data', msg_type=String, node_name: str = 'imu_subscriber', callback=None):
        default_callback = lambda msg: self.default_callback(msg)

        super().__init__(topic_name, msg_type, node_name, default_callback)
        QObject.__init__(self)
        self.parser = IMUDataParser()
        self.distance = 0.0
        self.velocity = 0.0
        self.vertical_tilt_angle = 0.0
        self.horizontal_tilt_angle = 0.0

    def default_callback(self, msg):
        """
        Overrides the default callback to parse the IMU message.

        The message is expected to be of type String, with the actual IMU data in msg.data.
        It computes the distance and velocity, then logs the results.
        """
        print("IMU Subscriber default callback called")
        try:
            parsed_data = self.parser.parse_imu_data(msg.data)
            # TODO: In the event that part of the IMU msg is missing, logger generated warning - is this even an issue?
            if len(parsed_data) == 4:
                self.distance, self.velocity, self.vertical_tilt_angle, self.horizontal_tilt_angle = parsed_data
                self.get_logger().info(f"IMU data parsed: distance = {self.distance:.2f}, velocity = {self.velocity:.2f}, vertical tilt = {self.vertical_tilt_angle:.2f}, horizontal tilt = {self.horizontal_tilt_angle:.2f}")
                self.imu_data_updated.emit(self.velocity, self.vertical_tilt_angle, self.horizontal_tilt_angle)
            else:
                self.get_logger().warning("Incomplete IMU data received.")
        except Exception as e:
            # Resets all the values to -0.1 in the event of an error (easier to detect vs 0.0)?
            self.distance = -0.1
            self.velocity = -0.1
            self.vertical_tilt_angle = -0.1
            self.horizontal_tilt_angle = -0.1
            self.get_logger().error(f"Failed to parse IMU data: {e}")

    @property
    def imu_distance(self):
        """
        Returns the distance traveled based on the last parsed IMU data.
        This property allows access to the distance without needing to call a method.
        """
        return self.distance
    @property
    def imu_velocity(self):
        """
        Returns the velocity based on the last parsed IMU data.
        This property allows access to the velocity without needing to call a method.
        """
        return self.velocity
    @property
    def imu_vertical_tilt_angle(self):
        """
        Returns the vertical tilt angle based on the last parsed IMU data.
        This property allows access to the vertical tilt angle without needing to call a method.
        """
        return self.vertical_tilt_angle
    @property
    def imu_horizontal_tilt_angle(self):
        """
        Returns the horizontal tilt angle based on the last parsed IMU data.
        This property allows access to the horizontal tilt angle without needing to call a method.
        """
        return self.horizontal_tilt_angle

class GPSSubscriber(GenericSubscriber):
    """
    A ROS2 subscriber that parses GPS messages using the GPSDataParser.

    This class overrides the default callback of the GenericSubscriber to parse
    the incoming GPS message and compute the average SNR.
    """
    def __init__(self, topic_name: str = 'gps_data', msg_type=String, node_name: str = 'gps_subscriber', callback=None):
        super().__init__(topic_name, msg_type, node_name, callback)
        self.parser = GPSDataParser()

    def default_callback(self, msg):
        """
        Overrides the default callback to parse the GPS message.

        The message is expected to be of type String, with the actual GPS data in msg.data.
        It computes the average SNR and logs it.
        """
        try:
            snr_values, average_snr, coordinates = self.parser.parse_message(msg.data)
            self.get_logger().info(f"GPS data parsed: SNR values = {snr_values}, Average SNR = {average_snr:.2f}, Coordinates = {coordinates}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse GPS data: {e}")


def main(args=None):
    """
    Main function to run the GenericSubscribers for Arduino bridge topics.

    This function initializes the ROS2 Python client library, creates a GenericSubscriber instance for each
    Arduino bridge topic, and spins them concurrently using a MultiThreadedExecutor.

    The topics subscribed to correspond to the following Arduino bridges:
        - MotorBridge: 'motor_data'
        - UltrasonicBridge: 'ultrasonic_data'
        - GPSBridge: 'gps_data'
        - IMUBridge: 'imu_data'
    """
    rclpy.init(args=args)
    rclpy.logging.get_logger('subscriber').info(">> subscriber.py main() starting")


    # Create subscribers for each Arduino bridge topic with unique node names
    motor_subscriber = GenericSubscriber("motor_data", String, node_name="motor_subscriber")
    # ultrasonic_subscriber = GenericSubscriber("ultrasonic_data", String, node_name="ultrasonic_subscriber")
    gps_subscriber = GPSSubscriber("gps_data", String, node_name="gps_subscriber")
    imu_subscriber = IMUSubscriber("imu_data", String, node_name="imu_subscriber")

    # Use a MultiThreadedExecutor to handle multiple nodes concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(motor_subscriber)
    # executor.add_node(ultrasonic_subscriber)
    executor.add_node(gps_subscriber)
    executor.add_node(imu_subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        motor_subscriber.get_logger().info("Subscribers stopped by user.")
    finally:
        # Destroy nodes and shutdown properly
        motor_subscriber.destroy_node()
        # ultrasonic_subscriber.destroy_node()
        gps_subscriber.destroy_node()
        imu_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
