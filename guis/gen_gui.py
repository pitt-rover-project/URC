# General GUI for the robot control system

# Import libraries
import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QFormLayout, QPushButton, QLabel, QGroupBox
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

        # Stores a QRectangle that represents the main window
        screen_dimensions = QApplication.primaryScreen().availableGeometry()
        screen_width = screen_dimensions.width()
        screen_height = screen_dimensions.height()

        # setGeometry sets the size and position of the main window - setGeometry(x, y, width, height)
        # x and y are the top-left corner coordinates
        self.setGeometry(0, 0, screen_width, screen_height)
        self.setWindowTitle("General GUI")

        # --- Attribute initialization ---
        # Buttons - Camera
        camera_feed_buttons = [
            QPushButton(f"Camera feed {i+1}", self) for i in range(3)
        ]
        camera_feed_buttons[2].setText("Camera feed 3 (for other thing)")

        # Buttons - Data display and control
        kill_button = QPushButton("Kill", self)
        imu_speed_on = QPushButton("IMU Speed On", self)
        imu_speed_off = QPushButton("IMU Speed Off", self)
        imu_orientation_on = QPushButton("IMU Orientation On", self)
        imu_orientation_off = QPushButton("IMU Orientation Off", self)
        gps_on = QPushButton("GPS On", self)
        gps_off = QPushButton("GPS Off", self)

        # Buttons - GUI controls
        gui_buttons = {
            "arduino_gui": QPushButton("Control", self),
            "auto_gui": QPushButton("Autonomous", self),
            "equip_serv_gui": QPushButton("Equipment Service", self),
            "ex_deli_gui": QPushButton("Extreme Delivery", self),
            "json_motorGUI": QPushButton("Motor and Arm", self),
        }

        # Labels - Data display
        data_display = {
            "imu_speed": QLabel(f"IMU Speed: {self.imu_attributes.imu_velocity}", self),
            "imu_orientation_vertical": QLabel(f"IMU Vertical Orientation: {self.imu_attributes.imu_vertical_tilt_angle}", self),
            "imu_orientation_horizontal": QLabel(f"IMU Horizontal Orientation: {self.imu_attributes.imu_horizontal_tilt_angle}", self),
            # "gps_data": QLabel(f"GPS Data: {self.imu_attributes.gps_data}", self)
        }

        # --- Connect GUI buttons to their corresponding actions ---
        gui_button_actions = {
            "arduino_gui": lambda: self.launch_gui("arduino_gui"),
            "auto_gui": lambda: self.launch_gui("auto_gui"),
            "equip_serv_gui": lambda: self.launch_gui("equip_serv_gui"),
            "ex_deli_gui": lambda: self.launch_gui("ex_deli_gui"),
            "json_motorGUI": lambda: self.launch_gui("json_motorGUI")
        }

        for key, button in gui_buttons.items():
            # Each button’s clicked signal triggers its mapped action.
            button.clicked.connect(gui_button_actions[key])

        # --- Layout setup ---
        # Camera feed
        camera_feed_buttons[0].setGeometry(5, 5, screen_width // 3, screen_height // 3)
        camera_feed_buttons[1].setGeometry(screen_width // 3 + 2, 5, screen_width // 3, screen_height // 3)
        camera_feed_buttons[2].setGeometry(2 * screen_width // 3, 5, screen_width // 3, screen_height // 3)

        # Data display and control buttons
        self.data_display = {
            "imu_speed": QLabel(f"IMU Speed: {self.imu_attributes.velocity}", self),
            "imu_orientation_vertical": QLabel(f"IMU Vertical Orientation: {self.imu_attributes.vertical_tilt_angle}", self),
            "imu_orientation_horizontal": QLabel(f"IMU Horizontal Orientation: {self.imu_attributes.horizontal_tilt_angle}", self),
        }
        imu_group = QGroupBox("IMU Data", self)
        imu_group.setGeometry(15, screen_height // 3 + 10, 600, 120)

        form = QFormLayout()
        form.addRow("Speed:",      self.data_display["imu_speed"])
        form.addRow("Vert. Tilt:", self.data_display["imu_orientation_vertical"])
        form.addRow("Horiz. Tilt:",self.data_display["imu_orientation_horizontal"])
        imu_group.setLayout(form)


        # data_display["imu_speed"].setGeometry(5, screen_height // 3, 200, 100)
        # data_display["imu_orientation_vertical"].setGeometry(screen_width // 3 + 5, screen_height // 3, 200, 100)
        # data_display["imu_orientation_horizontal"].setGeometry(2*screen_width // 3 + 5, screen_height // 3, 200, 100)

        # data_display["imu_speed"].setGeometry(0, screen_height // 3, 200, 100)
        # data_display["imu_orientation_vertical"].setGeometry(screen_width // 3, screen_height // 3, 200, 100)
        # data_display["imu_orientation_horizontal"].setGeometry(2*screen_width // 3, screen_height // 3, 200, 100)
        # gps_data_label = QLabel(f"GPS Data: {self.imu_attributes.gps_data}", self)

        # IMU control buttons
        imu_speed_on.setGeometry(0, screen_height // 3 + 60, 200, 50)
        imu_speed_off.setGeometry(200, screen_height // 3 + 60, 200, 50)

        # IMU orientation buttons
        imu_orientation_on.setGeometry(screen_width // 3, screen_height // 3 + 60, 200, 50)
        imu_orientation_off.setGeometry(screen_width // 3 + 200, screen_height // 3 + 60, 200, 50)

        # GPS control buttons
        gps_on.setGeometry(2*screen_width // 3, screen_height // 3 + 60, 200, 50)
        gps_off.setGeometry(2*screen_width // 3 + 200, screen_height // 3 + 60, 200, 50)

        # Kill button
        kill_button.setGeometry(screen_width // 3, screen_height // 8 * 7, 400, 50)

        # Placing the gui buttons
        gui_buttons["arduino_gui"].setGeometry(0, screen_height // 3 + 200, 200, 50)
        gui_buttons["auto_gui"].setGeometry(200, screen_height // 3 + 200, 200, 50)
        gui_buttons["equip_serv_gui"].setGeometry(0, screen_height // 3 + 300, 400, 50)
        gui_buttons["ex_deli_gui"].setGeometry(0, screen_height // 3 + 250, 200, 50)
        gui_buttons["json_motorGUI"].setGeometry(200, screen_height // 3 + 250, 200, 50)

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
