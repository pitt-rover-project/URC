import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QPushButton, QLabel
)
from PyQt5.QtCore import Qt
import subprocess


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.speed = 0
        self.ultrasonic_distances = [0, 0, 0]
        self.ultrasonic_labels = [QLabel("") for _ in range(3)]
        self.setup_ui()

    def setup_ui(self):
        screen_dimensions = QApplication.primaryScreen().availableGeometry()
        screen_width = screen_dimensions.width()
        screen_height = screen_dimensions.height()
        self.setGeometry(0, 0, screen_width, screen_height)
        self.setWindowTitle("General GUI")

        
        camera_feed_buttons = [
            QPushButton(f"camera feed {i+1}", self) for i in range(3)
        ]
        camera_feed_buttons[2].setText("camera feed 3 (for other thing)")

        kill_button = QPushButton("kill", self)
        imu_speed_on = QPushButton("IMU Speed On", self)
        imu_speed_off = QPushButton("IMU Speed Off", self)
        imu_orientation_on = QPushButton("IMU Orientation On", self)
        imu_orientation_off = QPushButton("IMU Orientation Off", self)
        gps_on = QPushButton("GPS On", self)
        gps_off = QPushButton("GPS Off", self)

        data_display = {
            "imu_speed": QLabel("imu speed", self),
            "imu_orientation": QLabel("imu orientation", self),
            "gps_data": QLabel("gps data", self)
        }

        gui_buttons = {
            "arduino_gui": QPushButton("Control", self),
            "auto_gui": QPushButton("Autonomous", self),
            "equip_serv_gui": QPushButton("Equipment Service", self),
            "ex_deli_gui": QPushButton("Extreme Delivery", self),
            "json_motorGUI": QPushButton("Motor and Arm", self),
        }

        gui_button_actions = {
            "arduino_gui": lambda: self.launch_gui("arduino_gui"),
            "auto_gui": lambda: self.launch_gui("auto_gui"),
            "equip_serv_gui": lambda: self.launch_gui("equip_serv_gui"),
            "ex_deli_gui": lambda: self.launch_gui("ex_deli_gui"),
            "json_motorGUI": lambda: self.launch_gui("json_motorGUI")
        }

        # Setting Everything on the window
        camera_feed_buttons[0].setGeometry(0, 0, screen_width // 3, screen_height // 3)
        camera_feed_buttons[1].setGeometry(screen_width // 3, 0, screen_width // 3, screen_height // 3)
        camera_feed_buttons[2].setGeometry(2 * screen_width // 3, 0, screen_width // 3, screen_height // 3)
        
        data_display["imu_speed"].setGeometry(0, screen_height // 3, 200, 100)
        data_display["imu_orientation"].setGeometry(screen_width // 3, screen_height // 3, 200, 100)
        data_display["gps_data"].setGeometry(2*screen_width // 3, screen_height // 3, 200, 100)

        imu_speed_on.setGeometry(0, screen_height // 3 + 60, 200, 50)
        imu_speed_off.setGeometry(200, screen_height // 3 + 60, 200, 50)

        imu_orientation_on.setGeometry(screen_width // 3, screen_height // 3 + 60, 200, 50)
        imu_orientation_off.setGeometry(screen_width // 3 + 200, screen_height // 3 + 60, 200, 50)

        gps_on.setGeometry(2*screen_width // 3, screen_height // 3 + 60, 200, 50)
        gps_off.setGeometry(2*screen_width // 3 + 200, screen_height // 3 + 60, 200, 50)

        kill_button.setGeometry(screen_width // 3 + 200, screen_height // 3 + 120, 200, 50)

    @staticmethod
    def launch_gui(name):
        subprocess.Popen(["python3", f"{name}.py"])


if __name__ == "__main__":
    app = QApplication(sys.argv)

    main_window = MainWindow()
    main_window.show()

    sys.exit(app.exec())