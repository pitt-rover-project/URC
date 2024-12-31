# import serial
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import *
from PyQt5 import *
from threading import Timer
import subprocess

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.speed = 0
        self.ultrasonic = [0, 0, 0]
        self.ultrasonicLabels = [QLabel(""), QLabel(""), QLabel("")]
        self.UI()

    def UI(self):
        self.setWindowTitle("General GUI")
        self.setGeometry(700, 300, 1200, 900)

        main_layout = QVBoxLayout()
        top_layout = QHBoxLayout()
        top_left_layout = QGridLayout()
        top_right_layout = QGridLayout()
        bottom_layout = QGridLayout()

        camera_feed_1 = QPushButton("camera feed 1")
        camera_feed_2 = QPushButton("camera feed 2")
        camera_feed_3 = QPushButton("camera feed 3 (for other thing)")
        ultrasonic_data = QPushButton("ultrasonic data") 
        imu_speed = QPushButton("imu speed")
        imu_orientation = QPushButton("imu orientation")
        gps_data = QPushButton("gpu data")
        kill_switch = QPushButton("kill")
        
        top_left_layout.addWidget(camera_feed_1, 0, 1)
        top_left_layout.addWidget(camera_feed_2, 0, 2)
        top_left_layout.addWidget(camera_feed_3, 0, 3)
        top_left_layout.addWidget(ultrasonic_data, 1, 2)
        top_left_layout.addWidget(imu_speed, 2, 1)
        top_left_layout.addWidget(imu_orientation, 2, 3)
        top_left_layout.addWidget(gps_data, 2, 2)
        top_left_layout.addWidget(kill_switch, 3, 2)

        arduino_gui = QPushButton("arduino_gui", self)
        arduino_gui.clicked.connect(self.arduino_gui)
        auto_gui = QPushButton("auto_gui", self)
        auto_gui.clicked.connect(self.auto_gui)
        equip_serv_gui = QPushButton("equip_serv_gui", self)
        equip_serv_gui.clicked.connect(self.equip_serv_gui)
        ex_deli_gui = QPushButton("ex_deli_gui", self)
        ex_deli_gui.clicked.connect(self.ex_deli_gui)
        json_motorGUI = QPushButton("json_motorGUI", self)
        json_motorGUI.clicked.connect(self.json_motorGUI)
        
        bottom_layout.addWidget(arduino_gui, 0, 0)
        bottom_layout.addWidget(auto_gui, 0, 1)
        bottom_layout.addWidget(equip_serv_gui, 0, 2)
        bottom_layout.addWidget(ex_deli_gui, 0, 3)
        bottom_layout.addWidget(json_motorGUI, 0, 4)

        top_layout.addLayout(top_left_layout)
        top_layout.addLayout(top_right_layout)
        main_layout.addLayout(top_layout)
        main_layout.addLayout(bottom_layout)

        self.setLayout(main_layout)
    
    def arduino_gui(self):
        subprocess.Popen(["python3", "arduino_gui.py"])
    
    def auto_gui(self):
        subprocess.Popen(["python3", "auto_gui.py"])
    
    def equip_serv_gui(self):
        subprocess.Popen(["python3", "equip_serv_gui.py"])
    
    def ex_deli_gui(self):
        subprocess.Popen(["python3", "ex_deli_gui.py"])
    
    def json_motorGUI_gui(self):
        subprocess.Popen(["python3", "json_motorGUI.py"])

if __name__ == "__main__":
    app = QApplication(sys.argv)

    main = MainWindow()
    main.show()

    sys.exit(app.exec())
