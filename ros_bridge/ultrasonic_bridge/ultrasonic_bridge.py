# This bridge uses the ros 2 python library and pyserial to communicate between the jetson nano and an arduino.

import rclpy  # Documentation: https://docs.ros2.org/latest/api/rclpy/index.html
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class ArduinoBridge(Node):
    def __init__(self):
        # Initialize the ROS2 node
        super().__init__("arduino_bridge")

        # Initialize the serial connection to the Arduino
        self.serial = serial.Serial("/dev/ttyACM0", 115200)  # Update with your port
        time.sleep(2)  # Wait for the Arduino to reset

        # Publisher to publish data read from the ultrasonic sensors
        self.ultrasonic_publisher = self.create_publisher(
            msg_type=String, 
            topic="ultrasonic_data", 
            qos_profile=10
        )

        # Timer to periodically read data from the Arduino
        self.create_timer(timer_period_sec=0.1, callback=self.read_from_arduino)

        self.get_logger().info("Arduino Bridge Node Started")

    def read_from_arduino(self):
        # Check if there's data available to read from the serial port
        if self.serial.in_waiting > 0:
            # Read a line of data from the Arduino
            response = self.serial.readline().decode("utf-8").strip()

            # Log the data for debugging
            self.get_logger().info(f"Received from Arduino: {response}")

            # Publish the data to the ROS 2 topic
            msg = String()
            msg.data = response
            self.publisher.publish(msg)


def main(args=None):
    # Initialize the ROS2 system
    rclpy.init(args=args)
    node = ArduinoBridge()

    try:
        # Start the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up the node and ROS2
        # ends the bridge
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()