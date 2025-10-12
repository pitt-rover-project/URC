#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import time


class MotorSubscriber(Node):
    def __init__(self, serial_port="/dev/ttyACM1", baud_rate=115200):
        super().__init__("motor_subscriber")

        # Initialize serial connection to Arduino
        self.serial = None
        try:
            self.serial = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Allow Arduino to reset
            self.get_logger().info(
                f"✅ Serial connection established on {serial_port} at {baud_rate} baud"
            )
        except serial.SerialException as e:
            self.get_logger().error(
                f"❌ Failed to connect to serial port {serial_port}: {e}"
            )
            self.get_logger().error(
                f"Motors will NOT respond! Check Arduino connection."
            )

        # Publisher for motor commands (for monitoring/debugging)
        self.publisher = self.create_publisher(String, "motor_commands", 10)

        # Publisher for Arduino feedback
        self.drive_data_publisher = self.create_publisher(String, "drive_data", 10)

        # Subscriber to receive motor control commands
        self.subscription = self.create_subscription(
            String, "motor_control_input", self.motor_command_callback, 10
        )

        # Subscriber to receive Twist messages and convert them
        self.twist_subscription = self.create_subscription(
            Twist, "cmd_vel", self.twist_callback, 10
        )

        self.get_logger().info("Motor Subscriber Node Started")
        self.get_logger().info("Subscribing to: motor_control_input and cmd_vel")
        self.get_logger().info(
            "Publishing to: motor_commands (debug) and drive_data (Arduino feedback)"
        )

        # Timer to read Arduino responses
        self.create_timer(0.1, self.read_from_arduino)

    def motor_command_callback(self, msg):
        """Handle incoming motor command strings and send to Arduino"""
        try:
            # Parse the incoming command and ensure it's in the correct format
            command_data = msg.data.strip()

            # If it's already in the correct format (6 comma-separated values), pass it through
            values = command_data.split(",")
            if len(values) == 6:
                # Validate that all values are numeric
                try:
                    [float(val) for val in values]
                    formatted_command = command_data
                except ValueError:
                    self.get_logger().warning(
                        f"Invalid numeric values in command: {command_data}"
                    )
                    return
            else:
                # If not in correct format, set to default
                formatted_command = "0,0,0,0,0,0"
                self.get_logger().warning(
                    f"Invalid command format, using default: {formatted_command}"
                )

            # CRITICAL: Write to Arduino serial
            if self.serial is not None and self.serial.is_open:
                serial_command = f"{formatted_command}\n"
                self.serial.write(serial_command.encode())
                self.get_logger().info(f"✅ Sent to Arduino: {formatted_command}")
            else:
                self.get_logger().error(
                    f"❌ Cannot send to Arduino: Serial not connected!"
                )

            # Publish for monitoring/debugging
            output_msg = String()
            output_msg.data = formatted_command
            self.publisher.publish(output_msg)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing motor command: {e}")

    def twist_callback(self, msg):
        """Convert Twist messages to motor command format and send to Arduino"""
        try:
            # Convert Twist to the 6-value format: linear_x,linear_y,linear_z,angular_x,angular_y,angular_z
            formatted_command = f"{msg.linear.x},{msg.linear.y},{msg.linear.z},{msg.angular.x},{msg.angular.y},{msg.angular.z}"

            # CRITICAL: Write to Arduino serial
            if self.serial is not None and self.serial.is_open:
                serial_command = f"{formatted_command}\n"
                self.serial.write(serial_command.encode())
                self.get_logger().info(f"✅ Sent Twist to Arduino: {formatted_command}")
            else:
                self.get_logger().error(
                    f"❌ Cannot send Twist to Arduino: Serial not connected!"
                )

            # Publish for monitoring/debugging
            output_msg = String()
            output_msg.data = formatted_command
            self.publisher.publish(output_msg)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error converting Twist message: {e}")

    def read_from_arduino(self):
        """Read responses from Arduino and publish to drive_data topic"""
        if self.serial is None or not self.serial.is_open:
            return

        try:
            if self.serial.in_waiting > 0:
                response = self.serial.readline().decode("utf-8").strip()
                if response:
                    self.get_logger().info(f"Arduino response: {response}")
                    msg = String()
                    msg.data = response
                    self.drive_data_publisher.publish(msg)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
        except UnicodeDecodeError as e:
            self.get_logger().warning(f"Failed to decode Arduino data: {e}")

    def destroy_node(self):
        """Clean up serial connection on shutdown"""
        if self.serial and self.serial.is_open:
            self.get_logger().info("Closing serial port...")
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


if __name__ == "__main__":
    main()
