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
    QGridLayout, QFormLayout, QPushButton, QLabel, QGroupBox,
    QFrame, QSizePolicy, QSpacerItem
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap, QFont, QPalette, QColor
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
        self.apply_styles()

    def setup_ui(self):
        # Get screen dimensions and set window to fill screen
        screen_dimensions = QApplication.primaryScreen().availableGeometry()
        screen_width = screen_dimensions.width()
        screen_height = screen_dimensions.height()

        # Window settings - fill entire screen
        self.setGeometry(0, 0, screen_width, screen_height)
        self.setWindowTitle("Robot Control System - Professional Interface")
        
        # Use proportional sizing that adapts better to different screen sizes
        
        # Title bar using proportional positioning
        title_frame = QFrame(self)
        title_frame.setObjectName("titleFrame")
        title_frame.setGeometry(int(screen_width * 0.01), int(screen_height * 0.01), 
                               int(screen_width * 0.98), int(screen_height * 0.08))
        
        title_layout = QHBoxLayout(title_frame)
        title_label = QLabel("ROBOT CONTROL SYSTEM")
        title_label.setObjectName("titleLabel")
        title_label.setAlignment(Qt.AlignLeft)
        
        status_indicator = QLabel("● ONLINE")
        status_indicator.setObjectName("statusOnline")
        status_indicator.setAlignment(Qt.AlignRight)
        
        title_layout.addWidget(title_label)
        title_layout.addWidget(status_indicator)
        
        # Interface Navigation as tabs - positioned between title and camera feeds
        nav_tab_frame = QFrame(self)
        nav_tab_frame.setObjectName("navTabFrame")
        nav_tab_frame.setGeometry(int(screen_width * 0.01), int(screen_height * 0.10), 
                                 int(screen_width * 0.98), int(screen_height * 0.06))
        
        nav_tab_layout = QHBoxLayout(nav_tab_frame)
        nav_tab_layout.setSpacing(5)
        
        gui_buttons = {
            "arduino_gui": QPushButton("Control Interface"),
            "auto_gui": QPushButton("Autonomous Mode"),
            "equip_serv_gui": QPushButton("Equipment Service"),
            "ex_deli_gui": QPushButton("Extreme Delivery"),
            "json_motorGUI": QPushButton("Motor and Arm Control"),
        }
        
        # Connect GUI buttons and style them as tabs
        gui_button_actions = {
            "arduino_gui": lambda: self.launch_gui("arduino_gui"),
            "auto_gui": lambda: self.launch_gui("auto_gui"),
            "equip_serv_gui": lambda: self.launch_gui("equip_serv_gui"),
            "ex_deli_gui": lambda: self.launch_gui("ex_deli_gui"),
            "json_motorGUI": lambda: self.launch_gui("json_motorGUI"),
        }

        # IMU Data display - moved to center of screen
        self.data_display = {
            "imu_speed": QLabel(f"IMU Speed: {self.imu_attributes.velocity}"),
            "imu_orientation_vertical": QLabel(f"IMU Vertical Orientation: {self.imu_attributes.vertical_tilt_angle}"),
            "imu_orientation_horizontal": QLabel(f"IMU Horizontal Orientation: {self.imu_attributes.horizontal_tilt_angle}"),
        }
        imu_group = QGroupBox("IMU Data")
        imu_form_layout = QFormLayout()
        imu_form_layout.addRow("Speed:", self.data_display["imu_speed"])
        imu_form_layout.addRow("Vert. Tilt:", self.data_display["imu_orientation_vertical"])
        imu_form_layout.addRow("Horiz. Tilt:", self.data_display["imu_orientation_horizontal"])
        imu_group.setLayout(imu_form_layout)

        for key, button in gui_buttons.items():
            button.setObjectName("navTabButton")
            button.setMinimumHeight(int(screen_height * 0.04))
            button.clicked.connect(gui_button_actions[key])
            nav_tab_layout.addWidget(button)
        
        nav_tab_layout.addStretch()  # Push tabs to the left
        
        # Camera section title
        camera_section_title = QLabel("Camera Feeds", self)
        camera_section_title.setObjectName("sectionTitle")
        camera_section_title.setGeometry(int(screen_width * 0.02), int(screen_height * 0.17), 
                                        int(screen_width * 0.2), int(screen_height * 0.04))
        
        # Calculate camera feed dimensions more proportionally
        camera_width = int(screen_width * 0.31)
        camera_height = int(screen_height * 0.35)
        camera_y_start = int(screen_height * 0.21)  # Moved down to accommodate tabs
        
        # Primary Camera Feed - better proportional sizing
        camera1_frame = QFrame(self)
        camera1_frame.setObjectName("cameraFrame")
        camera1_frame.setGeometry(int(screen_width * 0.02), camera_y_start, 
                                 camera_width, camera_height)
        
        camera1_layout = QVBoxLayout(camera1_frame)
        camera1_title = QLabel("Primary Camera Feed")
        camera1_title.setObjectName("cameraTitle")
        camera1_title.setAlignment(Qt.AlignCenter)
        camera1_layout.addWidget(camera1_title)
        
        # Single merged camera display
        self.camera_label.setParent(camera1_frame)
        self.camera_label.setObjectName("cameraDisplay")
        self.camera_label.setScaledContents(True)
        self.camera_label.setAlignment(Qt.AlignCenter)
        camera1_layout.addWidget(self.camera_label)
        
        # Camera feed 2 button - centered positioning
        camera2_frame = QFrame(self)
        camera2_frame.setObjectName("cameraFrame")
        camera2_frame.setGeometry(int(screen_width * 0.35), camera_y_start, 
                                 camera_width, camera_height)
        
        camera2_layout = QVBoxLayout(camera2_frame)
        camera2_title = QLabel("Camera Feed 2")
        camera2_title.setObjectName("cameraTitle")
        camera2_title.setAlignment(Qt.AlignCenter)
        camera2_layout.addWidget(camera2_title)
        
        camera2_btn = QPushButton("Camera feed 2")
        camera2_btn.setObjectName("cameraButton")
        camera2_layout.addWidget(camera2_btn)
        
        # Camera feed 3 button - right aligned
        camera3_frame = QFrame(self)
        camera3_frame.setObjectName("cameraFrame")
        camera3_frame.setGeometry(int(screen_width * 0.68), camera_y_start, 
                                 camera_width, camera_height)
        
        camera3_layout = QVBoxLayout(camera3_frame)
        camera3_title = QLabel("Auxiliary Feed")
        camera3_title.setObjectName("cameraTitle")
        camera3_title.setAlignment(Qt.AlignCenter)
        camera3_layout.addWidget(camera3_title)
        
        camera3_btn = QPushButton("Camera feed 3 (for other thing)")
        camera3_btn.setObjectName("cameraButton")
        camera3_layout.addWidget(camera3_btn)
        
        # Calculate control section positioning more proportionally
        controls_y_start = int(screen_height * 0.575)
        
        # IMU Data group - responsive sizing
        imu_group = QGroupBox("IMU Sensor Data", self)
        imu_group.setObjectName("dataGroup")
        imu_group.setGeometry(int(screen_width * 0.02), controls_y_start, 
                             int(screen_width * 0.45), int(screen_height * 0.2))
        
        imu_layout = QFormLayout(imu_group)
        self.data_display = {
            "imu_speed": QLabel("0.00 m/s"),
            "imu_orientation_vertical": QLabel("0.00°"),
            "imu_orientation_horizontal": QLabel("0.00°"),
        }
        
        for label in self.data_display.values():
            label.setObjectName("dataValue")
        
        imu_layout.addRow("Speed:", self.data_display["imu_speed"])
        imu_layout.addRow("Vertical Tilt:", self.data_display["imu_orientation_vertical"])
        imu_layout.addRow("Horizontal Tilt:", self.data_display["imu_orientation_horizontal"])
        
        # System Controls group - responsive sizing
        system_group = QGroupBox("System Controls", self)
        system_group.setObjectName("controlGroup")
        system_group.setGeometry(int(screen_width * 0.5), controls_y_start, 
                                int(screen_width * 0.47), int(screen_height * 0.2))
        
        # IMU control buttons - proportional positioning within system group
        imu_speed_on = QPushButton("IMU Speed On", self)
        imu_speed_off = QPushButton("IMU Speed Off", self)
        imu_orientation_on = QPushButton("IMU Orientation On", self)
        imu_orientation_off = QPushButton("IMU Orientation Off", self)
        gps_on = QPushButton("GPS On", self)
        gps_off = QPushButton("GPS Off", self)
        
        control_buttons = [imu_speed_on, imu_speed_off, imu_orientation_on, 
                          imu_orientation_off, gps_on, gps_off]
        for btn in control_buttons:
            btn.setObjectName("controlButton")
            btn.setMinimumHeight(int(screen_height * 0.04))
        
        # Position control buttons with better spacing
        button_width = int(screen_width * 0.2)
        button_height = int(screen_height * 0.04)
        controls_x_start = int(screen_width * 0.52)
        
        imu_speed_on.setGeometry(controls_x_start, controls_y_start + int(screen_height * 0.05), 
                               button_width, button_height)
        imu_speed_off.setGeometry(controls_x_start + button_width + 10, controls_y_start + int(screen_height * 0.05), 
                                 button_width, button_height)
        imu_orientation_on.setGeometry(controls_x_start, controls_y_start + int(screen_height * 0.10), 
                                     button_width, button_height)
        imu_orientation_off.setGeometry(controls_x_start + button_width + 10, controls_y_start + int(screen_height * 0.10), 
                                       button_width, button_height)
        gps_on.setGeometry(controls_x_start, controls_y_start + int(screen_height * 0.15), 
                          button_width, button_height)
        gps_off.setGeometry(controls_x_start + button_width + 10, controls_y_start + int(screen_height * 0.15), 
                           button_width, button_height)
        
        # GUI Navigation has been moved to tabs above - this section is now empty
        
        # Emergency controls - moved up since navigation tabs are now at top
        emergency_width = int(screen_width * 0.3)
        emergency_height = int(screen_height * 0.12)
        emergency_x = int((screen_width - emergency_width) / 2)
        emergency_y = int(screen_height * 0.80)  # Moved up from 0.85
        
        emergency_group = QGroupBox("Emergency Controls", self)
        emergency_group.setObjectName("emergencyGroup")
        emergency_group.setGeometry(emergency_x, emergency_y, emergency_width, emergency_height)
        
        emergency_layout = QVBoxLayout(emergency_group)
        kill_button = QPushButton("EMERGENCY STOP")
        kill_button.setObjectName("killButton")
        kill_button.setMinimumHeight(int(screen_height * 0.06))
        emergency_layout.addWidget(kill_button)

    def apply_styles(self):
        """Apply dark mode styling to the GUI"""
        style = """
        QWidget {
            background-color: #1e1e1e;
            color: #e0e0e0;
            font-family: 'Segoe UI', Arial, sans-serif;
            font-size: 12px;
        }
        
        #titleFrame {
            background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 0,
                                        stop: 0 #404040, stop: 1 #2d2d2d);
            border: 1px solid #555555;
            border-radius: 8px;
            padding: 10px;
            margin-bottom: 10px;
        }
        
        #titleLabel {
            color: #ffffff;
            font-size: 24px;
            font-weight: bold;
            letter-spacing: 2px;
        }
        
        #statusOnline {
            color: #4caf50;
            font-size: 14px;
            font-weight: bold;
        }
        
        #sectionFrame {
            background-color: #2a2a2a;
            border: 1px solid #444444;
            border-radius: 8px;
            padding: 15px;
            margin: 5px;
        }
        
        #sectionTitle {
            color: #ffffff;
            font-size: 16px;
            font-weight: bold;
            margin-bottom: 10px;
        }
        
        #navTabFrame {
            background-color: #2a2a2a;
            border: 1px solid #444444;
            border-radius: 8px;
            padding: 5px;
            margin: 2px;
        }
        
        #navTabButton {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #ff7043, stop: 1 #e64a19);
            border: none;
            border-radius: 6px;
            color: white;
            font-weight: bold;
            padding: 8px 15px;
            margin: 2px;
            min-width: 120px;
        }
        
        #navTabButton:hover {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #ff8a65, stop: 1 #ff7043);
        }
        
        #navTabButton:pressed {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #e64a19, stop: 1 #d84315);
        }
        
        #cameraFrame {
            background-color: #2a2a2a;
            border: 2px solid #444444;
            border-radius: 8px;
            padding: 10px;
        }
        
        #cameraTitle {
            color: #ffffff;
            font-size: 12px;
            font-weight: bold;
            margin-bottom: 5px;
        }
        
        #cameraDisplay, #grayImageDisplay {
            background-color: #1a1a1a;
            border: 2px solid #555555;
            border-radius: 4px;
            color: #cccccc;
            font-size: 14px;
        }
        
        #cameraButton {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #616161, stop: 1 #424242);
            border: none;
            border-radius: 6px;
            color: white;
            font-weight: bold;
            font-size: 11px;
        }
        
        #cameraButton:hover {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #757575, stop: 1 #616161);
        }
        
        #cameraButton:pressed {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #424242, stop: 1 #303030);
        }
        
        QGroupBox {
            font-size: 14px;
            font-weight: bold;
            color: #ffffff;
            border: 2px solid #444444;
            border-radius: 8px;
            margin-top: 10px;
            padding-top: 15px;
            background-color: #2a2a2a;
        }
        
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 15px;
            padding: 0 10px 0 10px;
            background-color: #2a2a2a;
        }
        
        #dataGroup {
            border: 2px solid #42a5f5;
        }
        
        #dataGroup::title {
            color: #42a5f5;
        }
        
        #controlGroup {
            border: 2px solid #ff7043;
        }
        
        #controlGroup::title {
            color: #ff7043;
        }
        
        #emergencyGroup {
            border: 2px solid #f44336;
        }
        
        #emergencyGroup::title {
            color: #f44336;
        }
        
        #dataValue {
            color: #66bb6a;
            font-size: 14px;
            font-weight: bold;
            background-color: #1a1a1a;
            border: 1px solid #444444;
            border-radius: 4px;
            padding: 5px;
            margin: 2px;
        }
        
        #controlButton {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #42a5f5, stop: 1 #1e88e5);
            border: none;
            border-radius: 6px;
            color: white;
            font-weight: bold;
            padding: 8px;
            margin: 2px;
        }
        
        #controlButton:hover {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #64b5f6, stop: 1 #42a5f5);
        }
        
        #controlButton:pressed {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #1e88e5, stop: 1 #1565c0);
        }
        
        #guiButton {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #ff7043, stop: 1 #e64a19);
            border: none;
            border-radius: 6px;
            color: white;
            font-weight: bold;
            padding: 10px;
            margin: 2px;
        }
        
        #guiButton:hover {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #ff8a65, stop: 1 #ff7043);
        }
        
        #guiButton:pressed {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #e64a19, stop: 1 #d84315);
        }
        
        #killButton {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #f44336, stop: 1 #d32f2f);
            border: none;
            border-radius: 8px;
            color: white;
            font-weight: bold;
            font-size: 16px;
            padding: 15px;
            margin: 2px;
        }
        
        #killButton:hover {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #e57373, stop: 1 #f44336);
        }
        
        #killButton:pressed {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #d32f2f, stop: 1 #b71c1c);
        }
        
        QFormLayout QLabel {
            color: #e0e0e0;
            font-weight: bold;
        }
        """
        
        self.setStyleSheet(style)

    def update_imu_display(self, velocity, vert_tilt, horiz_tilt):
        self.data_display["imu_speed"].setText(f"{velocity:.2f} m/s")
        self.data_display["imu_orientation_vertical"].setText(f"{vert_tilt:.2f}°")
        self.data_display["imu_orientation_horizontal"].setText(f"{horiz_tilt:.2f}°")

    def update_camera_display(self, frame):
        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        qimg = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)

    def update_gray_display(self, array):
        # array is expected to be a 2D numpy uint8 array (grayscale)
        # This now updates the same camera_label as the color camera
        try:
            arr = np.clip(array, 0, 255).astype(np.uint8)
            if arr.ndim != 2:
                # if a single-channel image came as HxWx1, squeeze it
                arr = arr.squeeze()
            height, width = arr.shape
            bytes_per_line = width
            q_image = QImage(arr.data, width, height, bytes_per_line, QImage.Format_Grayscale8)
            pixmap = QPixmap.fromImage(q_image)
            self.camera_label.setPixmap(pixmap)  # Using same label as color camera
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

    imu.imu_data_updated.connect(main_window.update_imu_display)

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
