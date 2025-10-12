#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import time


class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_subscriber')

        # Initialize serial connection to Arduino at /dev/ttyACM1
        try:
            # Use a very short timeout to avoid blocking
            self.serial = serial.Serial('/dev/ttyACM1', 115200, timeout=0.01)
            self.get_logger().info('Serial connection established on /dev/ttyACM1 at 115200 baud')
            # Note: Arduino reset happens automatically on serial open, but we don't wait for it
            # The first few commands might be missed, but subsequent ones will work immediately
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port /dev/ttyACM1: {e}')
            self.serial = None

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
        if self.serial is None:
            self.get_logger().warning('Cannot send command: serial connection not available')
            return

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

            # Send to Arduino via serial
            serial_command = f"{formatted_command}\n"
            self.serial.write(serial_command.encode())
            self.get_logger().info(f"Sent to Arduino: {formatted_command}")

            # Also publish the formatted command to ROS topic
            output_msg = String()
            output_msg.data = formatted_command
            self.publisher.publish(output_msg)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing motor command: {e}")

    def twist_callback(self, msg):
        """Convert Twist messages to motor command format"""
        if self.serial is None:
            self.get_logger().warning('Cannot send Twist command: serial connection not available')
            return

        try:
            # Convert Twist to the 6-value format: linear_x,linear_y,linear_z,angular_x,angular_y,angular_z
            formatted_command = f"{msg.linear.x},{msg.linear.y},{msg.linear.z},{msg.angular.x},{msg.angular.y},{msg.angular.z}"

            # Send to Arduino via serial
            serial_command = f"{formatted_command}\n"
            self.serial.write(serial_command.encode())
            self.get_logger().info(f"Sent Twist to Arduino: {formatted_command}")

            # Also publish the formatted command to ROS topic
            output_msg = String()
            output_msg.data = formatted_command
            self.publisher.publish(output_msg)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error converting Twist message: {e}")

    def destroy_node(self):
        """Clean up serial connection before shutting down"""
        if self.serial and self.serial.is_open:
            self.get_logger().info('Closing serial port...')
            try:
                self.serial.close()
            except serial.SerialException as e:
                self.get_logger().error(f"Error closing serial port: {e}")
        super().destroy_node()


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