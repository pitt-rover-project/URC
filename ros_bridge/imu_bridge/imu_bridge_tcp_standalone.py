#!/usr/bin/env python3
"""
Standalone TCP-based IMU Bridge for Docker containers on macOS.
This bridge connects to a TCP server (serial_tcp_bridge.py) running on the host
instead of directly accessing serial devices.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import time

class IMUBridgeTCPStandalone(Node):
    def __init__(self):
        super().__init__('imu_bridge_tcp_standalone')
        
        # Publisher for IMU data
        self.publisher = self.create_publisher(String, 'imu_data', 10)

        # Setup TCP connection to host
        self.tcp_host = "host.docker.internal"  # Docker Desktop's host gateway
        self.tcp_port = 8888
        self.tcp_socket = None
        self.connect_tcp()
        
        # Timer to periodically read from TCP
        self.create_timer(0.1, self.read_from_tcp)
        
        self.get_logger().info("IMU Bridge TCP Standalone Node Started")

    def connect_tcp(self):
        """Connect to the TCP bridge on the host"""
        try:
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_socket.settimeout(5)
            self.tcp_socket.connect((self.tcp_host, self.tcp_port))
            self.tcp_socket.settimeout(0.1)  # Set short timeout for reads
            self.get_logger().info(f"TCP connection established to {self.tcp_host}:{self.tcp_port}")
            return True
        except socket.error as e:
            self.get_logger().error(f"Failed to connect to TCP bridge: {e}")
            self.tcp_socket = None
            return False

    def read_from_tcp(self):
        """Read from TCP and publish to ROS topic"""
        if self.tcp_socket is None:
            self.get_logger().warning("TCP connection not available, attempting reconnection...")
            self.connect_tcp()
            return

        try:
            # Try to read data from TCP socket
            data = self.tcp_socket.recv(1024)
            if data:
                response = data.decode("utf-8").strip()
                if response:  # Only process non-empty responses
                    self.get_logger().info(f"Received from Arduino via TCP: {response}")
                    msg = String()
                    msg.data = response
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Published IMU data to /imu_data topic")
        except socket.timeout:
            # Timeout is normal when no data is available
            pass
        except socket.error as e:
            self.get_logger().error(f"TCP communication error: {e}")
            self._attempt_tcp_reconnection()
        except UnicodeDecodeError as e:
            self.get_logger().warning(f"Failed to decode TCP data: {e}")

    def _attempt_tcp_reconnection(self):
        """Attempt to reconnect to the TCP bridge"""
        if self.tcp_socket:
            try:
                self.tcp_socket.close()
            except:
                pass
            self.tcp_socket = None

        self.get_logger().warning("TCP connection lost. Attempting reconnection...")
        time.sleep(1)  # Wait before attempting reconnection
        self.connect_tcp()

    def destroy_node(self):
        """Override to close TCP socket"""
        if self.tcp_socket:
            self.get_logger().info("Closing TCP connection...")
            try:
                self.tcp_socket.close()
            except socket.error as e:
                self.get_logger().error(f"Error closing TCP socket: {e}")
        super().destroy_node()

def main(args=None):
    # Initialize the ROS2 system
    rclpy.init()
    node = IMUBridgeTCPStandalone()
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