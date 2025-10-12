#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_subscriber')

        # Publisher for motor commands in the specified format
        self.publisher = self.create_publisher(String, 'motor_commands', 10)

        # Subscriber to receive motor control commands
        self.subscription = self.create_subscription(
            String,
            'motor_control_input',
            self.motor_command_callback,
            10
        )

        # Subscriber to receive Twist messages and convert them
        self.twist_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )

        self.get_logger().info('Motor Subscriber Node Started')
        self.get_logger().info('Subscribing to: motor_control_input and cmd_vel')
        self.get_logger().info('Publishing to: motor_commands')

    def motor_command_callback(self, msg):
        """Handle incoming motor command strings"""
        try:
            # Parse the incoming command and ensure it's in the correct format
            command_data = msg.data.strip()

            # If it's already in the correct format (6 comma-separated values), pass it through
            values = command_data.split(',')
            if len(values) == 6:
                # Validate that all values are numeric
                try:
                    [float(val) for val in values]
                    formatted_command = command_data
                except ValueError:
                    self.get_logger().warning(f"Invalid numeric values in command: {command_data}")
                    return
            else:
                # If not in correct format, set to default
                formatted_command = "0,0,0,0,0,0"
                self.get_logger().warning(f"Invalid command format, using default: {formatted_command}")

            # Publish the formatted command
            output_msg = String()
            output_msg.data = formatted_command
            self.publisher.publish(output_msg)

            self.get_logger().info(f"Published motor command: {formatted_command}")

        except Exception as e:
            self.get_logger().error(f"Error processing motor command: {e}")

    def twist_callback(self, msg):
        """Convert Twist messages to motor command format"""
        try:
            # Convert Twist to the 6-value format: linear_x,linear_y,linear_z,angular_x,angular_y,angular_z
            formatted_command = f"{msg.linear.x},{msg.linear.y},{msg.linear.z},{msg.angular.x},{msg.angular.y},{msg.angular.z}"

            # Publish the formatted command
            output_msg = String()
            output_msg.data = formatted_command
            self.publisher.publish(output_msg)

            self.get_logger().info(f"Converted Twist to motor command: {formatted_command}")

        except Exception as e:
            self.get_logger().error(f"Error converting Twist message: {e}")


def main(args=None):
    rclpy.init(args=args)
    motor_subscriber = MotorSubscriber()

    try:
        rclpy.spin(motor_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        motor_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()