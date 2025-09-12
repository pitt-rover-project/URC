# General GUI for the robot control system
import sys
import os
from rclpy.node import Node

import numpy as np

from guis.subscribers.RGBsubcriber import ImageSubscriber
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
# Import libraries
import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QFormLayout, QPushButton, QLabel, QGroupBox
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
import subprocess
from pathlib import Path
from std_msgs.msg import String
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from PyQt5.QtGui import QImage, QPixmap
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


if 'QT_QPA_PLATFORM_PLUGIN_PATH' in os.environ:
    del os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']

# Import subscribers and publishers
import guis.subscribers.subscriber as sub

# Main window extends QWidget
class MainWindow(QWidget):
    # Constructor initializes the main window and sets up the UI
    def __init__(self, imu_attributes):
        super().__init__()
        # Initialize class attributes
        # Use the imu_attributes passed in (main passes a subscriber instance)
        self.imu_attributes = imu_attributes
        self.image_attributes = ImageSubscriber()         # Image subscriber instance
        self.speed = 0                                          # Initial speed of the robot
        self.ultrasonic_distances = [0, 0, 0]                   # Distances from ultrasonic sensors
        self.ultrasonic_labels = [QLabel("") for _ in range(3)] # Labels for ultrasonic sensors
        self.camera_label = QLabel(self)
        # Label to display grayscale image
        self.gray_image_label = QLabel(self)

        # This actually sets up the UI
        self.setup_ui()

    
    def setup_ui(self):
        # Stores a QRectangle that represents the main window
        screen_dimensions = QApplication.primaryScreen().availableGeometry()
        screen_width = screen_dimensions.width()
        screen_height = screen_dimensions.height()

        # Window settings
        self.setGeometry(0, 0, screen_width, screen_height)
        self.setWindowTitle("General GUI")

        # Camera preview label
        self.camera_label.setGeometry(5, 5, screen_width // 3, screen_height // 3)
        self.camera_label.setScaledContents(True)

        # Basic control buttons
        camera_feed_buttons = [QPushButton(f"Camera feed {i+1}", self) for i in range(3)]
        camera_feed_buttons[2].setText("Camera feed 3 (for other thing)")

        kill_button = QPushButton("Kill", self)
        imu_speed_on = QPushButton("IMU Speed On", self)
        imu_speed_off = QPushButton("IMU Speed Off", self)
        imu_orientation_on = QPushButton("IMU Orientation On", self)
        imu_orientation_off = QPushButton("IMU Orientation Off", self)
        gps_on = QPushButton("GPS On", self)
        gps_off = QPushButton("GPS Off", self)

        gui_buttons = {
            "arduino_gui": QPushButton("Control", self),
            "auto_gui": QPushButton("Autonomous", self),
            "equip_serv_gui": QPushButton("Equipment Service", self),
            "ex_deli_gui": QPushButton("Extreme Delivery", self),
            "json_motorGUI": QPushButton("Motor and Arm", self),
        }

        # Data display placeholders
        self.data_display = {
            "imu_speed": QLabel("IMU Speed: 0", self),
            "imu_orientation_vertical": QLabel("IMU Vertical Orientation: 0", self),
            "imu_orientation_horizontal": QLabel("IMU Horizontal Orientation: 0", self),
            "gray_image_display": QLabel(self),
        }

        # Position gray image widgets
        self.data_display["gray_image_display"].setGeometry(0, screen_height // 3 + 110, 200, 150)
        self.gray_image_label.setGeometry(210, screen_height // 3 + 110, 200, 150)
        self.gray_image_label.setScaledContents(True)

        # Connect GUI buttons
        gui_button_actions = {
            "arduino_gui": lambda: self.launch_gui("arduino_gui"),
            "auto_gui": lambda: self.launch_gui("auto_gui"),
            "equip_serv_gui": lambda: self.launch_gui("equip_serv_gui"),
            "ex_deli_gui": lambda: self.launch_gui("ex_deli_gui"),
            "json_motorGUI": lambda: self.launch_gui("json_motorGUI"),
        }

        for key, button in gui_buttons.items():
            button.clicked.connect(gui_button_actions[key])

        # Camera feed button positions
        camera_feed_buttons[0].setGeometry(5, 5, screen_width // 3, screen_height // 3)
        camera_feed_buttons[1].setGeometry(screen_width // 3 + 2, 5, screen_width // 3, screen_height // 3)
        camera_feed_buttons[2].setGeometry(2 * screen_width // 3, 5, screen_width // 3, screen_height // 3)

        # IMU group and layout
        imu_group = QGroupBox("IMU Data", self)
        imu_group.setGeometry(15, screen_height // 3 + 10, 600, 120)
        form = QFormLayout()
        form.addRow("Speed:",      self.data_display["imu_speed"])
        form.addRow("Vert. Tilt:", self.data_display["imu_orientation_vertical"])
        form.addRow("Horiz. Tilt:",self.data_display["imu_orientation_horizontal"])
        imu_group.setLayout(form)

        # IMU control buttons geometry
        imu_speed_on.setGeometry(0, screen_height // 3 + 60, 200, 50)
        imu_speed_off.setGeometry(200, screen_height // 3 + 60, 200, 50)
        imu_orientation_on.setGeometry(screen_width // 3, screen_height // 3 + 60, 200, 50)
        imu_orientation_off.setGeometry(screen_width // 3 + 200, screen_height // 3 + 60, 200, 50)
        gps_on.setGeometry(2*screen_width // 3, screen_height // 3 + 60, 200, 50)
        gps_off.setGeometry(2*screen_width // 3 + 200, screen_height // 3 + 60, 200, 50)

        # Kill button
        kill_button.setGeometry(screen_width // 3, screen_height // 8 * 7, 400, 50)

        # Place GUI buttons
        gui_buttons["arduino_gui"].setGeometry(0, screen_height // 3 + 200, 200, 50)
        gui_buttons["auto_gui"].setGeometry(200, screen_height // 3 + 200, 200, 50)
        gui_buttons["equip_serv_gui"].setGeometry(0, screen_height // 3 + 300, 400, 50)
        gui_buttons["ex_deli_gui"].setGeometry(0, screen_height // 3 + 250, 200, 50)
        gui_buttons["json_motorGUI"].setGeometry(200, screen_height // 3 + 250, 200, 50)

    def update_imu_display(self, velocity, vert_tilt, horiz_tilt):
        self.data_display["imu_speed"].setText(f"IMU Speed: {velocity:.2f}")
        self.data_display["imu_orientation_vertical"].setText(f"IMU Vertical Orientation: {vert_tilt:.2f}")
        self.data_display["imu_orientation_horizontal"].setText(f"IMU Horizontal Orientation: {horiz_tilt:.2f}")

    def update_camera_display(self, frame):
        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        qimg = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)

    def update_gray_display(self, array):
        # array is expected to be a 2D numpy uint8 array (grayscale)
        try:
            arr = np.clip(array, 0, 255).astype(np.uint8)
            if arr.ndim != 2:
                # if a single-channel image came as HxWx1, squeeze it
                arr = arr.squeeze()
            height, width = arr.shape
            bytes_per_line = width
            q_image = QImage(arr.data, width, height, bytes_per_line, QImage.Format_Grayscale8)
            pixmap = QPixmap.fromImage(q_image)
            self.gray_image_label.setPixmap(pixmap)
        except Exception as e:
            print(f"Failed to update gray display: {e}")


        
    @staticmethod
    def launch_gui(name):
        # Launches a GUI script in a new process - static method because it does not need access to instance variables
        script_path = Path(__file__).resolve().parent / f"{name}.py"
        subprocess.Popen([sys.executable, str(script_path)])



class CameraSubscriber(QObject):
    # Signal emits (frame, is_gray: bool). If is_gray True, frame is 2D uint8 array; else BGR color image
    image_updated = pyqtSignal(object, bool)

    def __init__(self, topic_name, node):
        super().__init__()
        self.bridge = CvBridge()
        self.node = node
        self.sub = node.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            # Try to detect encoding and request appropriate conversion
            encoding = msg.encoding if hasattr(msg, 'encoding') else ''
            if 'rgb' in encoding.lower() or 'bgr' in encoding.lower():
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.image_updated.emit(frame, False)
            else:
                # fallback to mono8
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                self.image_updated.emit(frame, True)
        except Exception as e:
            self.node.get_logger().error(f"Image conversion failed: {e}")



if __name__ == "__main__":
    class GuiNode(Node):
        def __init__(self):
            super().__init__('gui_node')

    rclpy.init()
    gui_node = GuiNode()

    app = QApplication(sys.argv)
    imu = sub.IMUSubscriber("imu_data", String, node_name="imu_subscriber")
    main_window = MainWindow(imu)

    camera_sub = CameraSubscriber("camera/gray/image_raw", gui_node)
    # Dispatch grayscale vs color frames to the appropriate update methods
    camera_sub.image_updated.connect(lambda frame, is_gray: main_window.update_gray_display(frame) if is_gray else main_window.update_camera_display(frame))

    # imu.imu_data_updated.connect(main_window.update_imu_display)


    # Use a ROS executor that doesn't block the Qt event loop
    executor = SingleThreadedExecutor()
    executor.add_node(imu)
    executor.add_node(gui_node)

    # Use a QTimer to periodically spin ROS events
    timer = QTimer()
    timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0))
    timer.start(10)  # 10ms interval is usually fine

    main_window.show()
    sys.exit(app.exec())

    # Cleanup
    executor.shutdown()
    imu.destroy_node()
    rclpy.shutdown()
    
    # Connecting buttons to publishers
    # Connecting buttons to subscribers
    # Update string headers to give actual values