import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import pyrealsense2 as rs
import cv2
import numpy as np

class GrayImagePublisher(Node):
    def __init__(self):
        super().__init__('gray_image_publisher')

        # Publisher setup
        self.publisher_ = self.create_publisher(Image, 'camera/gray/image_raw', 10)
        self.bridge = CvBridge()

        # RealSense setup
        self.pipeline = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.cfg)

        # Timer to publish at 30 Hz
        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Wait for frames
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            self.get_logger().warn("No color frame received.")
            return

        # Convert to grayscale
        color_image = np.asanyarray(color_frame.get_data())
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Convert to ROS Image message and publish
        msg = self.bridge.cv2_to_imgmsg(gray_image, encoding='mono8')
        self.publisher_.publish(msg)
        self.get_logger().info('Published gray image')

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GrayImagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
