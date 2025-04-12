#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TeleopStringPublisher(Node):
    def __init__(self):
        super().__init__('teleop_string_publisher')
        # Subscribe to Twist messages (commonly published by teleop on 'cmd_vel')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )
        # Publisher that outputs the Twist data in string form
        self.publisher = self.create_publisher(String, 'teleop_cmd_str', 10)
        self.get_logger().info("TeleopStringPublisher node started, subscribing to 'cmd_vel'")

    def twist_callback(self, msg: Twist):
        """
        Callback that triggers on receiving a Twist message.
        We convert the Twist data to a descriptive string and publish it.
        """
        # Convert Twist data to string
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        linear_z = msg.linear.z
        angular_x = msg.angular.x
        angular_y = msg.angular.y
        angular_z = msg.angular.z

        teleop_str = (
            f"Linear: (x={linear_x:.2f}, y={linear_y:.2f}, z={linear_z:.2f}); "
            f"Angular: (x={angular_x:.2f}, y={angular_y:.2f}, z={angular_z:.2f})"
        )
        
        # Publish as a String message
        string_msg = String(data=teleop_str)
        self.publisher.publish(string_msg)
        self.get_logger().info(f"Published teleop data as string: {teleop_str}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopStringPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
"""How to Use
1.	Add this script to your ROS2 Python package (e.g., inside a scripts/ or src/ folder) and make sure it has execution permissions (chmod +x teleop_string_publisher.py).
2.	Build your package (if you are using a typical ROS2 workspace, run `colcon build`).
3.	Launch your teleop node (e.g., teleop_twist_keyboard), which should publish geometry_msgs/msg/Twist messages to cmd_vel.
4.	Run this node (something like ros2 run your_package teleop_string_publisher), and it will subscribe to cmd_vel, convert the incoming Twist messages to strings, and publish them on the teleop_cmd_str topic.

"""
