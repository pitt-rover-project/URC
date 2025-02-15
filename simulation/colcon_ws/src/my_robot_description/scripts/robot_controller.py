#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotTeleopRelay(Node):
    def __init__(self):
        super().__init__('robot_teleop_relay')

        # Subscriber to teleop commands
        self.subscription = self.create_subscription(
            Twist,
            '/teleop_cmd_vel',  # Replace with the topic teleop publishes to
            self.teleop_callback,
            10
        )
        
        # Publisher to forward commands to Gazebo
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Robot Teleop Relay Node Started!")

    def teleop_callback(self, msg: Twist):
        """Callback to process received teleop commands."""
        # Log the received teleop command
        self.get_logger().info(f"Received Teleop Command: linear.x={msg.linear.x}, angular.z={msg.angular.z}")

        # Forward the received command to the Gazebo topic
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTeleopRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
