import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import pyrealsense2 as rs
import cv2
import numpy as np

class GrayAndDepthPublisher(Node):
    def __init__(self):
        super().__init__('gray_and_depth_publisher')
        
        # Publishers
        self.gray_pub = self.create_publisher(Image, 'camera/gray/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.detection_pub = self.create_publisher(String, 'camera/detections', 10)
        self.annotated_pub = self.create_publisher(Image, 'camera/annotated/image_raw', 10)
        
        self.bridge = CvBridge()
        
        # RealSense setup
        self.pipeline = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.cfg)
        
        # Object detection parameters
        self.min_contour_area = 500  # Minimum area to consider as object
        self.max_contour_area = 50000  # Maximum area to filter out entire image
        
        # Color ranges for environmental objects (HSV format)
        # These are basic ranges - tune them based on your environment
        self.color_ranges = {
            'rock': {
                'lower': np.array([0, 0, 50]),      # Gray/brown rocks
                'upper': np.array([180, 50, 150])
            },
            'tree': {
                'lower': np.array([35, 40, 40]),    # Green vegetation
                'upper': np.array([85, 255, 255])
            },
            'brown_object': {
                'lower': np.array([10, 50, 50]),    # Brown objects (dirt, bark)
                'upper': np.array([25, 255, 200])
            }
        }
        
        # Timer to publish at 30 Hz
        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def detect_objects(self, color_image, depth_image):
        """
        Detect objects using color segmentation and contour detection
        Returns list of detections with bounding boxes and classes
        """
        detections = []
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # Apply Gaussian blur to reduce noise
        hsv_image = cv2.GaussianBlur(hsv_image, (5, 5), 0)
        
        for obj_class, color_range in self.color_ranges.items():
            # Create mask for this color range
            mask = cv2.inRange(hsv_image, color_range['lower'], color_range['upper'])
            
            # Apply morphological operations to clean up mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter by area
                if self.min_contour_area < area < self.max_contour_area:
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculate center depth
                    cx, cy = x + w // 2, y + h // 2
                    depth_value = depth_image[cy, cx] if 0 <= cy < depth_image.shape[0] and 0 <= cx < depth_image.shape[1] else 0
                    depth_meters = depth_value * 0.001  # Convert mm to meters
                    
                    # Calculate confidence based on contour properties
                    perimeter = cv2.arcLength(contour, True)
                    circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
                    confidence = min(0.5 + circularity * 0.5, 0.95)
                    
                    detections.append({
                        'class': obj_class,
                        'bbox': (x, y, w, h),
                        'confidence': confidence,
                        'depth': depth_meters,
                        'area': area
                    })
        
        return detections
    
    def draw_detections(self, image, detections):
        """Draw bounding boxes and labels on image"""
        annotated = image.copy()
        
        colors = {
            'rock': (128, 128, 128),
            'tree': (0, 255, 0),
            'brown_object': (0, 100, 200)
        }
        
        for det in detections:
            x, y, w, h = det['bbox']
            color = colors.get(det['class'], (255, 255, 255))
            
            # Draw bounding box
            cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
            
            # Create label
            label = f"{det['class']}: {det['confidence']:.2f}"
            if det['depth'] > 0:
                label += f" ({det['depth']:.2f}m)"
            
            # Draw label background
            (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(annotated, (x, y - label_h - 10), (x + label_w, y), color, -1)
            
            # Draw label text
            cv2.putText(annotated, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return annotated
    
    def create_detection_msg(self, detections):
        """Create ROS String message with detection data in JSON format"""
        detection_data = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'frame_id': 'camera_frame',
            'detections': []
        }
        
        for det in detections:
            x, y, w, h = det['bbox']
            detection_data['detections'].append({
                'class': det['class'],
                'confidence': float(det['confidence']),
                'bbox': {
                    'x': int(x),
                    'y': int(y),
                    'width': int(w),
                    'height': int(h),
                    'center_x': int(x + w / 2),
                    'center_y': int(y + h / 2)
                },
                'depth_meters': float(det['depth']),
                'area': int(det['area'])
            })
        
        msg = String()
        msg.data = json.dumps(detection_data)
        return msg
    
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
        
        # Perform object detection
        detections = self.detect_objects(color_image, depth_image)
        
        # Create annotated image with detections
        annotated_image = self.draw_detections(color_image, detections)
        
        # Show images (for visualization)
        cv2.imshow("Grayscale", gray_image)
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )
        cv2.imshow("Depth", depth_colormap)
        cv2.imshow("Detections", annotated_image)
        
        if cv2.waitKey(1) & 0xFF == ord('x'):
            rclpy.shutdown()
        
        # Publish grayscale image
        gray_msg = self.bridge.cv2_to_imgmsg(gray_image, encoding='mono8')
        self.gray_pub.publish(gray_msg)
        
        # Publish depth image (16-bit, 1 channel)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        self.depth_pub.publish(depth_msg)
        
        # Publish detections
        if detections:
            detection_msg = self.create_detection_msg(detections)
            self.detection_pub.publish(detection_msg)
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            self.annotated_pub.publish(annotated_msg)
            
            self.get_logger().info(f'Published {len(detections)} detections')
        
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