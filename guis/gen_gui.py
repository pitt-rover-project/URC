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

        data_buttons = {
            "ultrasonic_data": QPushButton("ultrasonic data"),
            "imu_speed": QPushButton("imu speed"),
            "imu_orientation": QPushButton("imu orientation"),
            "gps_data": QPushButton("gps data"),
            "kill_switch": QPushButton("kill")
        }

        for i, button in enumerate(camera_feed_buttons, start=1):
            top_left_layout.addWidget(button, 0, i)

        top_left_layout.addWidget(data_buttons["ultrasonic_data"], 1, 2)
        top_left_layout.addWidget(data_buttons["imu_speed"], 2, 1)
        top_left_layout.addWidget(data_buttons["imu_orientation"], 2, 3)
        top_left_layout.addWidget(data_buttons["gps_data"], 2, 2)
        top_left_layout.addWidget(data_buttons["kill_switch"], 3, 2)

        gui_buttons = {
            "arduino_gui": QPushButton("arduino_gui"),
            "auto_gui": QPushButton("auto_gui"),
            "equip_serv_gui": QPushButton("equip_serv_gui"),
            "ex_deli_gui": QPushButton("ex_deli_gui"),
            "json_motorGUI": QPushButton("json_motorGUI")
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