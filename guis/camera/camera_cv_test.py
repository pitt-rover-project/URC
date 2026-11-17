import rclpy
from rclpy.node import Node
import threading

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import pyrealsense2 as rs
import cv2
import numpy as np

class GrayPublisher(Node):
    def __init__(self):
        super().__init__('gray_publisher')

        self.gray_pub = self.create_publisher(Image, 'camera/gray/image_raw', 10)
        self.bridge = CvBridge()

        self.pipeline = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        self.pipeline.start(self.cfg)

        self._lock = threading.Lock()
        self._latest_frame = None
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)

    def _capture_loop(self):
        while rclpy.ok():
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if color_frame:
                with self._lock:
                    self._latest_frame = np.asanyarray(color_frame.get_data())

    def timer_callback(self):
        with self._lock:
            if self._latest_frame is None:
                return
            frame = self._latest_frame.copy()

        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_msg = self.bridge.cv2_to_imgmsg(gray_image, encoding='mono8')
        self.gray_pub.publish(gray_msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GrayPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
