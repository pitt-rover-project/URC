import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.bridge = CvBridge()

        # Grayscale subscription
        self.gray_sub = self.create_subscription(
            Image,
            'camera/gray/image_raw',
            self.gray_callback,
            10
        )

        # Depth subscription
        self.depth_sub = self.create_subscription(
            Image,
            'camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.get_logger().info('Gray and Depth Image Subscribers started and listening...')

    def gray_callback(self, msg):
        try:
            gray_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            print("Received Grayscale Image:")
            print(gray_image)

            # Optional display
            # cv2.imshow("Gray Image", gray_image)
            # cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f"Gray CvBridge Error: {e}")

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            print("Received Depth Image:")
            print(depth_image)

            # Normalize and display for visualization
            # depth_vis = cv2.convertScaleAbs(depth_image, alpha=0.03)
            # depth_colormap = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            # cv2.imshow("Depth Image", depth_colormap)
            # cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f"Depth CvBridge Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
