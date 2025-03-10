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
        self.setWindowTitle("General GUI")
        self.setGeometry(700, 300, 1200, 900)

        main_layout = QVBoxLayout()
        top_layout = QHBoxLayout()
        top_left_layout = QGridLayout()
        bottom_layout = QGridLayout()

        camera_feed_buttons = [
            QPushButton(f"camera feed {i+1}") for i in range(3)
        ]
        camera_feed_buttons[2].setText("camera feed 3 (for other thing)")

        kill_button = QPushButton("kill")

        data_display = {
            "imu_speed": QLabel("imu speed"),
            "imu_orientation": QLabel("imu orientation"),
            "gps_data": QLabel("gps data")
        }

        for i, button in enumerate(camera_feed_buttons, start=1):
            top_left_layout.addWidget(button, 0, i)

        top_left_layout.addWidget(data_display["imu_speed"], 2, 1)
        top_left_layout.addWidget(data_display["imu_orientation"], 2, 3)
        top_left_layout.addWidget(data_display["gps_data"], 2, 2)
        top_left_layout.addWidget(kill_button, 3, 2)

        gui_buttons = {
            "arduino_gui": QPushButton("Control"),
            "auto_gui": QPushButton("Autonomous"),
            "equip_serv_gui": QPushButton("Equipment Service"),
            "ex_deli_gui": QPushButton("Extreme Delivery"),
            "json_motorGUI": QPushButton("Motor and Arm")
        }

        gui_button_actions = {
            "arduino_gui": lambda: self.launch_gui("arduino_gui"),
            "auto_gui": lambda: self.launch_gui("auto_gui"),
            "equip_serv_gui": lambda: self.launch_gui("equip_serv_gui"),
            "ex_deli_gui": lambda: self.launch_gui("ex_deli_gui"),
            "json_motorGUI": lambda: self.launch_gui("json_motorGUI")
        }

        for col, (name, button) in enumerate(gui_buttons.items()):
            bottom_layout.addWidget(button, 0, col)
            button.clicked.connect(gui_button_actions[name])

        top_layout.addLayout(top_left_layout)
        main_layout.addLayout(top_layout)
        main_layout.addLayout(bottom_layout)

        self.setLayout(main_layout)

    @staticmethod
    def launch_gui(name):
        subprocess.Popen(["python3", f"guis/{name}.py"])


if __name__ == "__main__":
    app = QApplication(sys.argv)

    main_window = MainWindow()
    main_window.show()

    sys.exit(app.exec())