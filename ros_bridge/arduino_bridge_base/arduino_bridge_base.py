#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
from geometry_msgs.msg import Twist


""" Parent class """
# Base class for Arduino bridges
class ArduinoBridgeBase(Node):
    def __init__(self, node_name, topic_name, serial_port, baud_rate=115200, timer_period=0.1):
        super().__init__(node_name)

        # Initialize the serial connection with error handling
        try:
            self.serial = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Allow the Arduino to reset
            self.get_logger().info(f"Serial connection established on {serial_port} at {baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port {serial_port}: {e}")
            self.serial = None
            return

        # Publisher for the specific topic
        self.publisher = self.create_publisher(String, topic_name, 10)

        # Timer to periodically read from Arduino
        self.create_timer(timer_period, self.read_from_arduino)

        self.get_logger().info(f"{node_name} Node Started")

    def read_from_arduino(self):
        if self.serial is None:
            return
            
        try:
            if self.serial.in_waiting > 0:
                response = self.serial.readline().decode("utf-8").strip()
                if response:  # Only process non-empty responses
                    self.get_logger().info(f"Received from Arduino: {response}")
                    msg = String()
                    msg.data = response
                    self.publisher.publish(msg)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
            self._attempt_reconnection()
        except UnicodeDecodeError as e:
            self.get_logger().warning(f"Failed to decode serial data: {e}")

    def _attempt_reconnection(self):
        """Attempt to reconnect to the serial port"""
        if self.serial:
            try:
                self.serial.close()
            except:
                pass
            self.serial = None
        
        # Could implement reconnection logic here if needed
        self.get_logger().warning("Serial connection lost. Manual restart required.")
            
    # Overrides the parent class's destroy_node method to close the serial port before shutting down node
    def destroy_node(self):
        if self.serial and self.serial.is_open:
            self.get_logger().info("Closing serial port...")
            try:
                self.serial.close()
            except serial.SerialException as e:
                self.get_logger().error(f"Error closing serial port: {e}")
        super().destroy_node()

""" Child Classes """
class MotorBridge(ArduinoBridgeBase):
    def __init__(self):
        super().__init__(
            node_name="motor_bridge",
            topic_name="drive_data",
            serial_port="/dev/ttyACM0",
            baud_rate=115200,
            timer_period=0.1
        )
        # Subscriber to receive teleoperation commands (Twist messages) from teleop node
        self.teleop_subscriber = self.create_subscription(
            Twist,
            "/teleop_cmd_vel",
            self.teleop_callback,
            10
        )
        # Subscriber to receive drive commands as strings
        self.drive_subscriber = self.create_subscription(
            String,
            "drive_arduino_commands",
            self.write_to_arduino,
            10
        )
        self.get_logger().info("Motor Bridge Node Started")

    def read_from_arduino(self):
        if self.serial.in_waiting > 0:
            response = self.serial.readline().decode("utf-8").strip()
            self.get_logger().info(f"Received from Arduino: {response}")
            msg = String()
            msg.data = response
            self.publisher.publish(msg)

    def write_to_arduino(self, msg):
        if self.serial is None:
            self.get_logger().warning("Cannot send command: serial connection not available")
            return
            
        try:
            command = msg.data
            self.serial.write(command.encode())
            self.get_logger().info(f"Sent to Arduino: {command}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send command to Arduino: {e}")
            self._attempt_reconnection()

    def teleop_callback(self, msg):
        if self.serial is None:
            self.get_logger().warning("Cannot send teleop command: serial connection not available")
            return
            
        try:
            # Format the Twist message into a comma-separated string and send to Arduino
            command = f"{msg.linear.x},{msg.linear.y},{msg.linear.z},{msg.angular.x},{msg.angular.y},{msg.angular.z}\n"
            self.serial.write(command.encode())
            self.get_logger().debug(f"Sent teleop command: {command.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send teleop command to Arduino: {e}")
            self._attempt_reconnection()

# Ultrasonic Bridge Class
class UltrasonicBridge(ArduinoBridgeBase):
    def __init__(self, serial_port="/dev/ttyACM0", baud_rate=115200):
        super().__init__(
            node_name="ultrasonic_bridge",
            topic_name="ultrasonic_data",
            serial_port=serial_port,
            baud_rate=baud_rate
        )

# GPS Bridge
class GPSBridge(ArduinoBridgeBase):
    def __init__(self, serial_port="/dev/ttyACM0", baud_rate=115200):
        super().__init__(
            node_name="gps_bridge",
            topic_name="gps_data",
            serial_port=serial_port,
            baud_rate=baud_rate
        )

# IMU Bridge
class IMUBridge(ArduinoBridgeBase):
    def __init__(self):
        super().__init__(
            node_name="imu_bridge",
            topic_name="imu_data",
            serial_port="/dev/ttyACM0",
            baud_rate=115200
        )
