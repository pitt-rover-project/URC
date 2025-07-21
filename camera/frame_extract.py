import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RealSenseRGBPublisher(Node):
    def main(args=None):
        rclpy.init(args=args)
        node = RealSenseRGBPublisher()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def __init__(self):
        super().__init__('realsense_rgb_publisher')
        # Topic, outgoing messages 10 before they are dropped
        self.publisher_ = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.bridge = CvBridge()

        # RealSense pipeline, capture the image on this object
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # Publish at 30 Hz
        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Grab a frame
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            self.get_logger().warn("No color frame received.")
            return

        # publish it
        color_image = np.asanyarray(color_frame.get_data())
        # rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB) if we need to convert to rgb (uncomment this code)
        msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()
