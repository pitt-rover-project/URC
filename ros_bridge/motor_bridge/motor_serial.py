import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import threading # To handle the terminal input without blocking the main ROS 2 loop
from geometry_msgs.msg import Twist


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

        self.teleop_subscriber = self.create_subscription(
            msg_type=Twist,
            topic="/teleop_cmd_vel",
            callback=self.teleop_callback,
            qos_profile=10,
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
            self.drive_publisher.publish(msg)

    def write_to_arduino(self, msg):
        # Extract the command from the received ROS 2 message
        command = msg.data

        # Write the command to the Arduino via serial
        self.serial.write(command.encode())

        # Log the command sent to the Arduino
        self.get_logger().info(f"Sent to Arduino: {command}")
    
    def teleop_callback(self, msg: Twist):
        self.serial.write(f"{msg.linear.x},{msg.linear.y},{msg.linear.z},{msg.angular.x},{msg.angular.y},{msg.angular.z}\n")

class TerminalInputPublisher(Node):
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__("terminal_input_publisher")

        # Publisher to send user input to the `drive_arduino_commands` topic
        self.command_publisher = self.create_publisher(String, "drive_arduino_commands", 10)

        self.get_logger().info("Terminal Input Publisher Node Started")
        threading.Thread(target=self.publish_user_input, daemon=True).start()

    def publish_user_input(self):
        while rclpy.ok():
            try:
                # Continuously prompt the user for input
                user_input = input("Enter command for Arduino: ").strip()

                if user_input:
                    # Publish the input to the topic
                    msg = String()
                    msg.data = user_input
                    self.command_publisher.publish(msg)

                    self.get_logger().info(f"Published: {user_input}")
            except Exception as e:
                self.get_logger().error(f"Error reading terminal input: {e}")

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