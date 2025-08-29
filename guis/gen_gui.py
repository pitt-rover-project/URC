# General GUI for the robot control system

# Import libraries
import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QFormLayout, QPushButton, QLabel, QGroupBox, QSpacerItem, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer
import subprocess
from pathlib import Path
from std_msgs.msg import String
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor


# Import subscribers and publishers
import guis.subscribers.subscriber as sub

# Main window extends QWidget
class MainWindow(QWidget):
    # Constructor initializes the main window and sets up the UI
    def __init__(self, imu_attributes):
        super().__init__()
        # Initialize class attributes
        self.imu_attributes = imu_attributes             # IMU data parser instance
        self.imu_attributes.imu_data_updated.connect(self.update_imu_display)
        self.speed = 0                                          # Initial speed of the robot
        self.ultrasonic_distances = [0, 0, 0]                   # Distances from ultrasonic sensors
        self.ultrasonic_labels = [QLabel("") for _ in range(3)] # Labels for ultrasonic sensors

        # This actually sets up the UI
        self.setup_ui()

    def setup_ui(self):
        # Window setup - responsive without hardcoded dimensions
        self.setWindowTitle("General GUI")
        self.showMaximized()  # Responsive full-screen

        # --- Widget Creation ---
        # Camera feed buttons
        camera_feed_buttons = [
            QPushButton(f"Camera feed {i+1}") for i in range(3)
        ]
        camera_feed_buttons[2].setText("Camera feed 3 (for other thing)")

        # Control buttons
        kill_button = QPushButton("Kill")
        imu_speed_on = QPushButton("IMU Speed On")
        imu_speed_off = QPushButton("IMU Speed Off")
        imu_orientation_on = QPushButton("IMU Orientation On")
        imu_orientation_off = QPushButton("IMU Orientation Off")
        gps_on = QPushButton("GPS On")
        gps_off = QPushButton("GPS Off")

        # GUI navigation buttons
        gui_buttons = {
            "arduino_gui": QPushButton("Control"),
            "auto_gui": QPushButton("Autonomous"),
            "equip_serv_gui": QPushButton("Equipment Service"),
            "ex_deli_gui": QPushButton("Extreme Delivery"),
            "json_motorGUI": QPushButton("Motor and Arm"),
        }


        # --- Connect GUI buttons to their corresponding actions ---
        gui_button_actions = {
            "arduino_gui": lambda: self.launch_gui("arduino_gui"),
            "auto_gui": lambda: self.launch_gui("auto_gui"),
            "equip_serv_gui": lambda: self.launch_gui("equip_serv_gui"),
            "ex_deli_gui": lambda: self.launch_gui("ex_deli_gui"),
            "json_motorGUI": lambda: self.launch_gui("json_motorGUI")
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
            button.clicked.connect(gui_button_actions[key])

        # --- Responsive Layout Setup ---
        main_layout = QVBoxLayout()

        # Top section: Camera feeds
        camera_layout = QHBoxLayout()
        for camera_button in camera_feed_buttons:
            camera_button.setMinimumHeight(200)
            camera_layout.addWidget(camera_button)
        main_layout.addLayout(camera_layout)

        # Add spacing
        main_layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        # Middle section: IMU data (centered)
        imu_layout = QHBoxLayout()
        imu_layout.addSpacerItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        imu_layout.addWidget(imu_group)
        imu_layout.addSpacerItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        main_layout.addLayout(imu_layout)

        # Add spacing
        main_layout.addSpacerItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

        # Control buttons section
        control_layout = QGridLayout()
        control_layout.addWidget(imu_speed_on, 0, 0)
        control_layout.addWidget(imu_speed_off, 0, 1)
        control_layout.addWidget(imu_orientation_on, 0, 2)
        control_layout.addWidget(imu_orientation_off, 0, 3)
        control_layout.addWidget(gps_on, 0, 4)
        control_layout.addWidget(gps_off, 0, 5)
        main_layout.addLayout(control_layout)

        # GUI navigation buttons section
        gui_layout = QGridLayout()
        gui_layout.addWidget(gui_buttons["arduino_gui"], 0, 0)
        gui_layout.addWidget(gui_buttons["auto_gui"], 0, 1)
        gui_layout.addWidget(gui_buttons["ex_deli_gui"], 1, 0)
        gui_layout.addWidget(gui_buttons["json_motorGUI"], 1, 1)
        gui_layout.addWidget(gui_buttons["equip_serv_gui"], 2, 0, 1, 2)  # Span 2 columns
        main_layout.addLayout(gui_layout)

        # Kill button at bottom (centered)
        kill_layout = QHBoxLayout()
        kill_layout.addSpacerItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        kill_button.setMinimumHeight(50)
        kill_button.setMaximumWidth(400)
        kill_layout.addWidget(kill_button)
        kill_layout.addSpacerItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        main_layout.addLayout(kill_layout)

        # Set the main layout
        self.setLayout(main_layout)

    def update_imu_display(self, velocity, vert_tilt, horiz_tilt):
        self.data_display["imu_speed"].setText(f"IMU Speed: {velocity:.2f}")
        self.data_display["imu_orientation_vertical"].setText(f"IMU Vertical Orientation: {vert_tilt:.2f}")
        self.data_display["imu_orientation_horizontal"].setText(f"IMU Horizontal Orientation: {horiz_tilt:.2f}")


    @staticmethod
    def launch_gui(name):
        # Launches a GUI script in a new process - static method because it does not need access to instance variables
        script_path = Path(__file__).resolve().parent / f"{name}.py"
        subprocess.Popen([sys.executable, str(script_path)])


if __name__ == "__main__":
    rclpy.init()

    app = QApplication(sys.argv)
    imu = sub.IMUSubscriber("imu_data", String, node_name="imu_subscriber")
    main_window = MainWindow(imu)

    imu.imu_data_updated.connect(main_window.update_imu_display)

    main_window.show()

    # Use a ROS executor that doesn't block the Qt event loop
    executor = SingleThreadedExecutor()
    executor.add_node(imu)

    # Use a QTimer to periodically spin ROS events
    timer = QTimer()
    timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0))
    timer.start(10)  # 10ms interval is usually fine

    sys.exit(app.exec())

    # Cleanup
    executor.shutdown()
    imu.destroy_node()
    rclpy.shutdown()

    # Connecting buttons to publishers
    # Connecting buttons to subscribers
    # Update string headers to give actual values
