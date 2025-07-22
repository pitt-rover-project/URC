import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import pyrealsense2 as rs
import cv2
import numpy as np

class GrayAndDepthPublisher(Node):
    def __init__(self):
        super().__init__('gray_and_depth_publisher')

        # Publishers
        self.gray_pub = self.create_publisher(Image, 'camera/gray/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.bridge = CvBridge()

        # RealSense setup
        self.pipeline = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.cfg)

        # Timer to publish at 30 Hz
        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Wait for frames
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            self.get_logger().warn("Missing color or depth frame.")
            return

        # Convert color to grayscale
        color_image = np.asanyarray(color_frame.get_data())
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Convert depth to NumPy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Show images (for visualization)
        cv2.imshow("Grayscale", gray_image)

        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )
        cv2.imshow("Depth", depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord('x'):
            rclpy.shutdown()

        # Publish grayscale image
        gray_msg = self.bridge.cv2_to_imgmsg(gray_image, encoding='mono8')
        self.gray_pub.publish(gray_msg)

        # Publish depth image (16-bit, 1 channel)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        self.depth_pub.publish(depth_msg)

        self.get_logger().info('Published gray and depth images')

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GrayAndDepthPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
