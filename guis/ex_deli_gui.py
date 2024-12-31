#import serial
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import *
from PyQt5 import *
from threading import Timer

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.speed = 0
        self.ultrasonic = [0, 0, 0]
        self.ultrasonicLabels = [QLabel(""), QLabel(""), QLabel("")]
        self.UI()

    def UI(self):
        self.setWindowTitle("Extreme Delivery GUI")
        self.setGeometry(700, 300, 1200, 900)

        main_layout = QVBoxLayout()
        top_layout = QHBoxLayout()
        top_left_layout = QGridLayout()
        top_right_layout = QGridLayout()
        bottom_layout = QGridLayout()

        chassis_control = QPushButton("control chassis")
        arm_control = QPushButton("control arm")
        weak_signal = QPushButton("rover automatically travels to gps location and overrides user control, to be implemented")

        top_left_layout.addWidget(chassis_control, 0, 1)
        top_left_layout.addWidget(arm_control, 0, 2)
        top_left_layout.addWidget(weak_signal, 1, 1)
        
        top_layout.addLayout(top_left_layout)
        top_layout.addLayout(top_right_layout)
        main_layout.addLayout(top_layout)
        main_layout.addLayout(bottom_layout)

        self.setLayout(main_layout)

if __name__ == "__main__":
    app = QApplication(sys.argv)

    main = MainWindow()
    main.show()

    sys.exit(app.exec())