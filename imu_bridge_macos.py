#!/usr/bin/env python3
"""
macOS-compatible IMU Bridge that connects to Arduino on /dev/cu.usbmodem11201
and publishes to ROS2 topics that can be accessed by the Docker container.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class IMUBridgeMacOS(Node):
    def __init__(self):
        super().__init__('imu_bridge_macos')

        # Use the macOS device path
        serial_port = '/dev/cu.usbmodem11201'
        baud_rate = 9600

        # Initialize the serial connection with error handling
        try:
            self.serial = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Allow the Arduino to reset
            self.get_logger().info(f"Serial connection established on {serial_port} at {baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port {serial_port}: {e}")
            self.serial = None
            return

        # Publisher for IMU data
        self.publisher = self.create_publisher(String, 'imu_data', 10)

        # Timer to periodically read from Arduino
        self.create_timer(0.1, self.read_from_arduino)

        self.get_logger().info("IMU Bridge macOS Node Started")

    def read_from_arduino(self):
        if self.serial is None:
            self.get_logger().info("IMU Bridge macOS: Serial connection not available")
            return

        try:
            if self.serial.in_waiting > 0:
                self.get_logger().info("IMU Bridge macOS: Reading from Arduino...")
                response = self.serial.readline().decode("utf-8").strip()
                self.get_logger().debug(f"Raw response: {response}")
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

        self.get_logger().warning("Serial connection lost. Manual restart required.")

    def destroy_node(self):
        if self.serial and self.serial.is_open:
            self.get_logger().info("Closing serial port...")
            try:
                self.serial.close()
            except serial.SerialException as e:
                self.get_logger().error(f"Error closing serial port: {e}")
        super().destroy_node()

def main(args=None):
    # Initialize the ROS2 system
    rclpy.init()
    node = IMUBridgeMacOS()
    try:
        # Start the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up the node and ROS2
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
