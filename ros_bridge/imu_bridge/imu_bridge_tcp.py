#!/usr/bin/env python3
"""
TCP-based IMU Bridge for Docker containers on macOS.
This bridge connects to a TCP server (serial_tcp_bridge.py) running on the host
instead of directly accessing serial devices.
"""

import rclpy
from ros_bridge.arduino_bridge_base.arduino_bridge_base import ArduinoBridgeBase
import socket
import time

class IMUBridgeTCP(ArduinoBridgeBase):
    def __init__(self):
        # Use a dummy serial port for the parent class
        super().__init__(
            node_name="imu_bridge_tcp",
            topic_name="imu_data", 
            serial_port="/dev/null",  # Dummy port
            baud_rate=9600
        )
        
        # Close the dummy serial connection
        if self.serial:
            self.serial.close()
            self.serial = None
            
        # Setup TCP connection to host
        self.tcp_host = "host.docker.internal"  # Docker Desktop's host gateway
        self.tcp_port = 8888
        self.tcp_socket = None
        self.connect_tcp()

    def connect_tcp(self):
        """Connect to the TCP bridge on the host"""
        try:
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_socket.settimeout(5)
            self.tcp_socket.connect((self.tcp_host, self.tcp_port))
            self.tcp_socket.settimeout(1)  # Set timeout for reads
            self.get_logger().info(f"TCP connection established to {self.tcp_host}:{self.tcp_port}")
        except socket.error as e:
            self.get_logger().error(f"Failed to connect to TCP bridge: {e}")
            self.tcp_socket = None

    def read_from_arduino(self):
        """Override to read from TCP instead of serial"""
        if self.tcp_socket is None:
            self.get_logger().info("IMU Bridge TCP: TCP connection not available")
            return

        try:
            # Set socket to non-blocking mode for polling
            self.tcp_socket.setblocking(False)
            try:
                data = self.tcp_socket.recv(1024)
                if data:
                    response = data.decode("utf-8").strip()
                    self.get_logger().info(f"Raw TCP response: {response}")
                    if response:  # Only process non-empty responses
                        self.get_logger().info(f"Received from Arduino via TCP: {response}")
                        from std_msgs.msg import String
                        msg = String()
                        msg.data = response
                        self.publisher.publish(msg)
                        self.get_logger().info(f"Published IMU data: {response}")
            except socket.error as e:
                if e.errno != 11 and e.errno != 35:  # EAGAIN/EWOULDBLOCK - no data available
                    self.get_logger().error(f"TCP communication error: {e}")
                    self._attempt_tcp_reconnection()
            finally:
                # Reset to blocking mode
                self.tcp_socket.setblocking(True)
        except socket.error as e:
            self.get_logger().error(f"TCP socket error: {e}")
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
    node = IMUBridgeTCP()
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