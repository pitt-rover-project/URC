import rclpy
from rclpy.node import Node

class TopicVisualizer(Node):
    def __init__(self):
        super().__init__('topic_visualizer')
        self.get_logger().info("Active Topics:")
        for topic in self.get_topic_names_and_types():
            self.get_logger().info(f"{topic}")

rclpy.init()
node = TopicVisualizer()
rclpy.spin_once(node)
node.destroy_node()
rclpy.shutdown()

