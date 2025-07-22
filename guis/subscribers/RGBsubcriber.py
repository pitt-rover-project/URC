import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np


class GrayImageSubscriber(Node):
    def __init__(self):
        super().__init__('gray_image_subscriber')

        self.subscription = self.create_subscription(
            Image,
            'camera/gray/image_raw',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('Gray Image Subscriber started and listening...')

    def listener_callback(self, msg):
        try:
            # Convert ROS Image to NumPy array
            gray_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            # Print grayscale array (or a subset)
            print("Received Gray Image:")
            print(gray_image)

            # Optional: display it (for visual debugging)
            # import cv2
            # cv2.imshow("Received Gray", gray_image)
            # cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GrayImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
