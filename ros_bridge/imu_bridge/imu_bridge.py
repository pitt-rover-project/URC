# This bridge uses the ros 2 python library and pyserial to communicate between the jetson nano and an arduino.
import rclpy
from ros_bridge.arduino_bridge_base.arduino_bridge_base import IMUBridge

def main(args=None):
    # Initialize the ROS2 system
    rclpy.init()
    node = IMUBridge()
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
