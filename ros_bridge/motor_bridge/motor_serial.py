from ros_bridge.arduino_bridge_base import MotorBridge
from rlcpy import init, spin, shutdown

def main(args=None):
    # Initialize the ROS2 system
    rclpy.init(args=args)
    node = MotorBridge()
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
