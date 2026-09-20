#!/usr/bin/env python3
"""
Generic ROS2 Publisher Class for Motor Commands

This module contains a generic publisher class that can be used to publish motor commands
to ROS2 topics from GUI applications. It follows the established patterns from the
subscribers module and integrates with PyQt5 for GUI applications.

The MotorPublisher specifically publishes motor commands in the format "0,0,0,0,0,0"
as requested, representing 6 motor values separated by commas.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from PyQt5.QtCore import QObject, pyqtSignal, QTimer
import threading


class GenericPublisher(Node):
    """
    A generic ROS2 publisher that publishes messages to a given topic.

    This class extends the rclpy Node class and can be used as a base class for creating
    ROS2 topic publishers from GUI applications.
    """

    def __init__(self, topic_name: str, msg_type, node_name: str = 'generic_publisher', queue_size: int = 10):
        """
        Initialize the GenericPublisher.

        Args:
            topic_name (str): The name of the ROS2 topic to publish to.
            msg_type (Type): The message type of the ROS2 topic.
            node_name (str, optional): The name of the ROS2 node. Defaults to 'generic_publisher'.
            queue_size (int, optional): The queue size for the publisher. Defaults to 10.
        """
        super().__init__(node_name)
        self.topic_name = topic_name
        self.msg_type = msg_type

        # Create a publisher for the specified topic
        self.publisher = self.create_publisher(msg_type, topic_name, queue_size)
        self.get_logger().info(f"Publisher created for topic: {topic_name}")

    def publish_message(self, message):
        """
        Publish a message to the topic.

        Args:
            message: The message to publish (should be of the correct message type).
        """
        try:
            self.publisher.publish(message)
            self.get_logger().debug(f"Published message to {self.topic_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish message: {e}")


class MotorPublisher(GenericPublisher, QObject):
    """
    A ROS2 publisher specifically for motor commands that integrates with PyQt5.

    This class publishes motor commands in the format "0,0,0,0,0,0" to the
    'motor_control_input' topic, which can be consumed by the motor subscriber
    in the ros_bridge.
    """

    message_published = pyqtSignal(str)  # Signal emitted when a message is published

    def __init__(self, topic_name: str = 'motor_control_input', node_name: str = 'motor_publisher'):
        """
        Initialize the MotorPublisher.

        Args:
            topic_name (str, optional): The topic to publish to. Defaults to 'motor_control_input'.
            node_name (str, optional): The ROS2 node name. Defaults to 'motor_publisher'.
        """
        GenericPublisher.__init__(self, topic_name, String, node_name)
        QObject.__init__(self)

        # Initialize with default motor values
        self.current_motor_values = [0, 0, 0, 0, 0, 0]

    def publish_motor_command(self, motor_values=None):
        """
        Publish motor command in the format "0,0,0,0,0,0".

        Args:
            motor_values (list, optional): List of 6 motor values. If None, uses default [0,0,0,0,0,0].
        """
        if motor_values is None:
            motor_values = [0, 0, 0, 0, 0, 0]

        # Validate input
        if not isinstance(motor_values, (list, tuple)) or len(motor_values) != 6:
            self.get_logger().warning(f"Invalid motor_values format. Expected list of 6 values, got: {motor_values}")
            motor_values = [0, 0, 0, 0, 0, 0]

        # Convert all values to strings and join with commas
        try:
            command_string = ",".join(str(val) for val in motor_values)

            # Create and publish the message
            msg = String()
            msg.data = command_string
            self.publish_message(msg)

            # Update current values and emit signal
            self.current_motor_values = list(motor_values)
            self.message_published.emit(command_string)

            self.get_logger().info(f"Published motor command: {command_string}")

        except Exception as e:
            self.get_logger().error(f"Failed to publish motor command: {e}")

    def set_motor_value(self, motor_index: int, value):
        """
        Set a specific motor value and publish the updated command.

        Args:
            motor_index (int): Index of the motor (0-5).
            value: The value to set for the motor.
        """
        if 0 <= motor_index < 6:
            self.current_motor_values[motor_index] = value
            self.publish_motor_command(self.current_motor_values)
        else:
            self.get_logger().warning(f"Invalid motor index: {motor_index}. Must be 0-5.")

    def set_all_motors(self, value):
        """
        Set all motor values to the same value and publish.

        Args:
            value: The value to set for all motors.
        """
        motor_values = [value] * 6
        self.publish_motor_command(motor_values)

    def stop_all_motors(self):
        """
        Stop all motors by setting all values to 0.
        """
        self.set_all_motors(0)

    def get_current_values(self):
        """
        Get the current motor values.

        Returns:
            list: Current motor values.
        """
        return self.current_motor_values.copy()


class TwistPublisher(GenericPublisher, QObject):
    """
    A ROS2 publisher for Twist messages that integrates with PyQt5.

    This class publishes geometry_msgs/Twist messages for robot movement control.
    """

    twist_published = pyqtSignal(float, float, float, float, float, float)  # Signal for published twist

    def __init__(self, topic_name: str = 'cmd_vel', node_name: str = 'twist_publisher'):
        """
        Initialize the TwistPublisher.

        Args:
            topic_name (str, optional): The topic to publish to. Defaults to 'cmd_vel'.
            node_name (str, optional): The ROS2 node name. Defaults to 'twist_publisher'.
        """
        GenericPublisher.__init__(self, topic_name, Twist, node_name)
        QObject.__init__(self)

    def publish_twist(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """
        Publish a Twist message with the specified velocities.

        Args:
            linear_x (float): Linear velocity in x direction (forward/backward).
            linear_y (float): Linear velocity in y direction (left/right).
            linear_z (float): Linear velocity in z direction (up/down).
            angular_x (float): Angular velocity around x axis (roll).
            angular_y (float): Angular velocity around y axis (pitch).
            angular_z (float): Angular velocity around z axis (yaw).
        """
        try:
            twist_msg = Twist()
            twist_msg.linear.x = float(linear_x)
            twist_msg.linear.y = float(linear_y)
            twist_msg.linear.z = float(linear_z)
            twist_msg.angular.x = float(angular_x)
            twist_msg.angular.y = float(angular_y)
            twist_msg.angular.z = float(angular_z)

            self.publish_message(twist_msg)
            self.twist_published.emit(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)

            self.get_logger().info(f"Published Twist: linear=({linear_x}, {linear_y}, {linear_z}), angular=({angular_x}, {angular_y}, {angular_z})")

        except Exception as e:
            self.get_logger().error(f"Failed to publish Twist message: {e}")

    def move_forward(self, speed=1.0):
        """Move the robot forward at the specified speed."""
        self.publish_twist(linear_x=speed)

    def move_backward(self, speed=1.0):
        """Move the robot backward at the specified speed."""
        self.publish_twist(linear_x=-speed)

    def turn_left(self, angular_speed=1.0):
        """Turn the robot left at the specified angular speed."""
        self.publish_twist(angular_z=angular_speed)

    def turn_right(self, angular_speed=1.0):
        """Turn the robot right at the specified angular speed."""
        self.publish_twist(angular_z=-angular_speed)

    def stop(self):
        """Stop the robot by publishing zero velocities."""
        self.publish_twist()


def main(args=None):
    """
    Main function to test the publishers.

    This function demonstrates how to use the MotorPublisher and TwistPublisher classes.
    """
    rclpy.init(args=args)

    # Create motor publisher instance
    motor_pub = MotorPublisher()

    try:
        # Test publishing default motor command
        motor_pub.publish_motor_command()

        # Test publishing custom motor values
        motor_pub.publish_motor_command([1, 2, 3, 4, 5, 6])

        # Test setting individual motor
        motor_pub.set_motor_value(0, 10)

        # Test stopping all motors
        motor_pub.stop_all_motors()

        # Keep the node alive for a bit
        rclpy.spin_once(motor_pub, timeout_sec=1.0)

    except KeyboardInterrupt:
        motor_pub.get_logger().info("Motor publisher stopped by user.")
    finally:
        motor_pub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
