#!/usr/bin/env python3
"""
Unified Motor Bridge Launcher

This script launches the enhanced MotorBridge that can handle both:
- Twist messages from cmd_vel topic (for teleop)
- String messages from motor_control_input topic (for GUI control)

Both input types are sent to the same Arduino serial connection.
"""

import rclpy
import sys
import os

# Add the parent directory to the path so we can import the bridge
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'arduino_bridge_base'))

from arduino_bridge_base import MotorBridge


def main(args=None):
    """Launch the unified motor bridge"""
    rclpy.init(args=args)

    try:
        motor_bridge = MotorBridge()
        motor_bridge.get_logger().info("Unified Motor Bridge started successfully")
        motor_bridge.get_logger().info("Listening for commands on:")
        motor_bridge.get_logger().info("  - cmd_vel (Twist messages)")
        motor_bridge.get_logger().info("  - motor_control_input (String messages)")
        motor_bridge.get_logger().info("  - drive_arduino_commands (String messages)")

        rclpy.spin(motor_bridge)

    except KeyboardInterrupt:
        motor_bridge.get_logger().info("Motor Bridge stopped by user")
    except Exception as e:
        motor_bridge.get_logger().error(f"Motor Bridge error: {e}")
    finally:
        if 'motor_bridge' in locals():
            motor_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()