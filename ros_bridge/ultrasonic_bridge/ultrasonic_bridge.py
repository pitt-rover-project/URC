# This bridge uses the ros 2 python library and pyserial to communicate between the jetson nano and an arduino.
from rclpy import init, spin, shutdown
from ros_bridge.arduino_bridge_base import ArduinoBridge

def main(args=None):
    # Initialize the ROS2 system
    rclpy.init(args=args)
    node = ArduinoBridge()

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