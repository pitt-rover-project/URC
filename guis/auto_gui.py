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
        self.setWindowTitle("Autonomous GUI")
        self.setGeometry(700, 300, 1200, 900)

        main_layout = QVBoxLayout()
        top_layout = QHBoxLayout()
        top_left_layout = QGridLayout()
        top_right_layout = QGridLayout()
        bottom_layout = QGridLayout()

        gnss = QPushButton("gnss")
        aruco = QPushButton("aruco")
        object_loc = QPushButton("object")
        message_signals = QPushButton("signal that constantly pulses messages to verify location")
        abort_button = QPushButton("abort the mission")

        top_left_layout.addWidget(gnss, 0, 1)
        top_left_layout.addWidget(aruco, 0, 2)
        top_left_layout.addWidget(object_loc, 0, 3)
        top_left_layout.addWidget(message_signals, 1, 2)
        top_left_layout.addWidget(abort_button, 2, 2)

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

