# Documentation: https://docs.ros2.org/latest/api/rclpy/index.html
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

""" Parent class """
# Base class for Arduino bridges
class ArduinoBridgeBase(Node):
    def __init__(self, node_name, topic_name, serial_port, baud_rate=9600, timer_period=0.1):
        super().__init__(node_name)

        # Initialize the serial connection
        self.serial = serial.Serial(serial_port, baud_rate)
        time.sleep(2)  # Allow the Arduino to reset

        # Publisher for the specific topic
        self.publisher = self.create_publisher(String, topic_name, 10)

        # Timer to periodically read from Arduino
        self.create_timer(timer_period, self.read_from_arduino)

        self.get_logger().info(f"{node_name} Node Started")

    def read_from_arduino(self):
        if self.serial.in_waiting > 0:
            response = self.serial.readline().decode("utf-8").strip()
            self.get_logger().info(f"Received from Arduino: {response}")
            msg = String()
            msg.data = response
            self.publisher.publish(msg)
            
""" Child Classes """
class MotorBridge(ArduinoBridgeBase):
    def __init__(self):
        super().__init__(
            node_name="motor_bridge",
            topic_name="motor_data",
            serial_port="/dev/ttyACM0",
            baud_rate=115200
        )
        
# Ultrasonic Bridge Class
class UltrasonicBridge(ArduinoBridgeBase):
    def __init__(self):
        super().__init__(
            node_name="ultrasonic_bridge",
            topic_name="ultrasonic_data",
            serial_port="/dev/ttyACM0",
            baud_rate=115200
        )
        
# GPS Bridge
class GPSBridge(ArduinoBridgeBase):
    def __init__(self):
        super().__init__(
            node_name="gps_bridge",
            topic_name="gps_data",
            serial_port="/dev/ttyACM0",
            baud_rate=9600
        )

# IMU Bridge
class IMUBridge(ArduinoBridgeBase):
    def __init__(self):
        super().__init__(
            node_name="imu_bridge",
            topic_name="imu_data",
            serial_port="/dev/ttyACM0",
            baud_rate=9600
        )
