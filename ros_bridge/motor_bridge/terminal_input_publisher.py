import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TerminalInputPublisher(Node):
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__("terminal_input_publisher")

        # Create a publisher to the "arduino_commands" topic
        self.publisher = self.create_publisher(String, "arduino_commands", 10)

        self.get_logger().info("Terminal Input Publisher Node Started")
        self.publish_user_input()

    def publish_user_input(self):
        try:
            while rclpy.ok():
                # Read user input from the terminal
                user_input = input("Enter command for Arduino: ").strip()

                if user_input:
                    # Create and publish a message with the user input
                    msg = String()
                    msg.data = user_input
                    self.publisher.publish(msg)

                    self.get_logger().info(f"Published: {user_input}")
        except KeyboardInterrupt:
            self.get_logger().info("Terminal Input Publisher Node Stopped")
            rclpy.shutdown()


def main(args=None):
    # Initialize the ROS 2 system
    rclpy.init(args=args)
    node = TerminalInputPublisher()

    try:
        # Keep the node running
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up the node and ROS 2
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()