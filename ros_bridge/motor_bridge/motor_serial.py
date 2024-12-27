import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time


class ArduinoBridge(Node):
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__("arduino_bridge")

        # Initialize the serial connection to the Arduino
        self.serial = serial.Serial("/dev/ttyACM0", 9600)  # Update with your port
        time.sleep(2)  # Wait for the Arduino to reset

        # Publisher and Subscriber are in different topics to separate the communication flows
            # Publisher publishes data from the Arduino to a ROS 2 topic
            # Subscriber receives data from a ROS 2 topic and sends it to the Arduino

        # Publisher to publish data read from the Arduino
        self.drive_publisher = self.create_publisher(
            msg_type=String, topic="drive_data", qos_profile=10
        )

        # Subscriber to receive data and send it to the Arduino
        # the driver_subscriber will need to receive terminal commands to send the drive instructions via terminal or a publisher
            # ros2 topic pub /drive_arduino_commands std_msgs/String "{data: 'your_command'}"
                # basically just publishes a message to the drive_arduino_commands topic via terminal
            # or run the terinal_input_publisher.py to send commands via terminal
        self.drive_subscriber = self.create_subscription(
            msg_type=String,
            topic="drive_arduino_commands",
            callback=self.write_to_arduino,
            qos_profile=10,
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

    def write_to_arduino(self, msg):
        # Extract the command from the received ROS 2 message
        command = msg.data

        # Write the command to the Arduino via serial
        self.serial.write(command.encode())

        # Log the command sent to the Arduino
        self.get_logger().info(f"Sent to Arduino: {command}")


def main(args=None):
    # Initialize the ROS 2 system
    rclpy.init(args=args)
    node = ArduinoBridge()

    try:
        # Start the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up the node and ROS 2
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()