#import serial
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import *
from PyQt5 import *
from threading import Timer
import math
import random


# https://stackoverflow.com/a/13151299
class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer = None
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False


# Top-left - motor controls
# Top-right - arm controls
# Bottom - sensors/cameras
# on and off for motors and arm
# buttons and keyboard control for motor and arms


# commPort = "/dev/ttyACM0"
# serMotor = serial.Serial(commPort, baudrate=115200)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.speed = 0
        self.ultrasonic = [0, 0, 0]
        self.ultrasonicLabels = [QLabel(""), QLabel(""), QLabel("")]
        self.UI()

    def UI(self):
        self.setWindowTitle("URC Gui")
        self.setGeometry(1600, 0, 1200, 900)

        main_layout = QVBoxLayout()
        top_layout = QHBoxLayout()
        top_left_layout = QGridLayout()
        top_right_layout = QGridLayout()
        bottom_layout = QGridLayout()

        forward_button = QPushButton("Forward\nI")
        forward_button.clicked.connect(self.forwardButton)
        left_button = QPushButton("Left\nJ")
        left_button.clicked.connect(self.leftButton)
        backwards_button = QPushButton("Backwards\n,")
        backwards_button.clicked.connect(self.backwardsButton)
        right_button = QPushButton("Right\nL")
        right_button.clicked.connect(self.rightButton)
        speed_up_button = QPushButton("Speed Up\nQ")
        speed_up_button.clicked.connect(self.speedupButton)
        slow_down_button = QPushButton("Slow Down\nZ")
        slow_down_button.clicked.connect(self.slowdownButton)
        stop_button = QPushButton("Stop\nK")
        stop_button.clicked.connect(self.stopButton)
        enable_button = QPushButton("Enable")
        disable_button = QPushButton("Disable")

        self.le1 = QLabel("Set Speed (UP/DOWN): " + str(self.speed))
        self.btn2 = QPushButton("Enter an integer")
        self.btn2.clicked.connect(self.getint)

        top_left_layout.addWidget(forward_button, 0, 1)
        top_left_layout.addWidget(left_button, 1, 0)
        top_left_layout.addWidget(backwards_button, 1, 1)
        top_left_layout.addWidget(right_button, 1, 2)
        top_left_layout.addWidget(speed_up_button, 2, 0)
        top_left_layout.addWidget(slow_down_button, 2, 2)
        top_left_layout.addWidget(stop_button, 3, 0)
        top_left_layout.addWidget(enable_button, 4, 1)
        top_left_layout.addWidget(disable_button, 3, 2)

        top_left_layout.addWidget(self.le1, 2, 1)
        top_left_layout.addWidget(self.btn2, 3, 1)

        top_right_layout.addWidget(QLabel("arm controls here"), 0, 0)
        # bottom_layout.addWidget(QLabel("cameras/sensors here"), 0, 0)

        # for i in range(len(self.ultrasonic)):
        #     bottom_layout.addWidget(self.ultrasonicLabels[i], i, 0)

        # rt = RepeatedTimer(0.1, self.getUltraSonicData)

        top_layout.addLayout(top_left_layout)
        top_layout.addLayout(top_right_layout)
        main_layout.addLayout(top_layout)
        main_layout.addLayout(bottom_layout)

        self.setLayout(main_layout)

    # commPort = "/dev/ttyACM2"
    # serMotor = serial.Serial(commPort, baudrate=115200)
    # def getUltraSonicData(self):
    #     while(serMotor.in_waiting > 0):
    #         read = serMotor.readline().decode("utf-8")
    #         # 32,43,54
    #         # in centimeters
    #         # read = (
    #         #    str(math.floor(random.random() * 100))
    #         #    + ","
    #         #    + str(math.floor(random.random() * 100))
    #         #    + ","
    #         #    + str(math.floor(random.random() * 100))
    #         # )

    #         read = read.split(",")

    #         res = []
    #         for i in range(3):
    #             res.append(float(read[i]))

    #         self.ultrasonic = res

    #         for i in range(len(self.ultrasonic)):
    #             self.ultrasonicLabels[i].setText(
    #                 "Ultrasonic " + str(i) + ": " + str(self.ultrasonic[i])
    #             )

    def getint(self):
        num, ok = QInputDialog.getInt(self, "integer input dualog", "enter a number (10-255)")
        if ok:
            self.speed = num
            self.le1.setText("Set Speed (UP/DOWN): " + str(self.speed))

    def forwardButton(self, event):
        self.motorEvent("I")

    def leftButton(self, event):
        self.motorEvent("J")

    def backwardsButton(self, event):
        self.motorEvent(",")

    def rightButton(self, event):
        self.motorEvent("L")

    def speedupButton(self, event):
        self.motorEvent("Q")

    def slowdownButton(self, event):
        self.motorEvent("Z")

    def stopButton(self, event):
        self.motorEvent("K")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_I:
            self.motorEvent("I")
        if event.key() == Qt.Key_Comma:
            self.motorEvent(",")
        if event.key() == Qt.Key_J:
            self.motorEvent("J")
        if event.key() == Qt.Key_L:
            self.motorEvent("L")
        if event.key() == Qt.Key_Q:
            self.motorEvent("Q")
        if event.key() == Qt.Key_Z:
            self.motorEvent("Z")
        if event.key() == Qt.Key_K:
            self.motorEvent("K")

    def motorEvent(self, keyEvent):
        if keyEvent == "I":
            print("Foward")
            self.foward()
        if keyEvent == ",":
            print("backwards")
            self.backward()
        if keyEvent == "J":
            print("Left")
            self.left()
        if keyEvent == "L":
            print("Right")
            self.right()
        if keyEvent == "Q":
            print("Speed Up")
            self.speed_up()
        if keyEvent == "Z":
            print("Slow Down")
            self.slow_down()
        if keyEvent == "K":
            print("stop")
            self.stop()

    def foward(self):
        serMotor.write("1".encode())
        return

    def backward(self):
        serMotor.write("2".encode())
        return

    def left(self):
        serMotor.write("3".encode())
        return

    def right(self):
        serMotor.write("4".encode())
        return

    def speed_up(self):
        # serMotor.write("5".encode())
        serMotor.write(str(self.speed).encode())
        # serMotor.write("150".encode())
        return

    def slow_down(self):
        # serMotor.write("6".encode())
        serMotor.write(str(self.speed).encode())
        return

    def stop(self):
        serMotor.write("7".encode())
        # serMotor.write(self.speed.encode())
        return


"""
# commPort2 = '/dev/ttyACM1'
# serArm = serial.Serial(commPort2, baudrate = 115200)
class ArmWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.UI_Arm()

    def UI_Arm(self):
        self.setWindowTitle("Arm Control")
        self.setGeometry(1600, 950, 600, 350)

        self.text = QLabel("Claw Opened = X || Claw Closed = C", self)
        self.text2 = QLabel("Base Shift Right = D || Base Shift Left = A", self)
        self.text3 = QLabel("Fowards:", self)
        self.text4 = QLabel(
            "Bottom Joint = U || Middle Joint = I || Top Joint = O", self
        )
        self.text5 = QLabel("Backwards: ", self)
        self.text6 = QLabel(
            "Bottom Joint = J || Middle Joint = K || Top Joint = L", self
        )
        self.text7 = QLabel("Wrist Clockwise = Y || Wrist Counterclockwise = H", self)
        self.text8 = QLabel("Stop All = Escape", self)

        self.text.setFont(QFont("Arial", 15))
        self.text.move(50, 25)
        self.text2.setFont(QFont("Arial", 15))
        self.text2.move(50, 75)
        self.text3.setFont(QFont("Arial", 15))
        self.text3.move(50, 120)
        self.text4.setFont(QFont("Arial", 13))
        self.text4.move(50, 140)
        self.text5.setFont(QFont("Arial", 15))
        self.text5.move(50, 170)
        self.text6.setFont(QFont("Arial", 13))
        self.text6.move(50, 190)
        self.text7.setFont(QFont("Arial", 15))
        self.text7.move(50, 230)
        self.text8.setFont(QFont("Arial", 15))
        self.text8.move(50, 260)

        closeButton = QPushButton("Close", self)
        closeButton.clicked.connect(self.close_on_click)
        closeButton.resize(150, 50)
        closeButton.move(225, 275)

    def close_on_click(self):
        self.close()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_0:
            print("Claw Opened")
            # self.claw_open()
        if event.key() == Qt.Key_9:
            print("Claw Closed")
            # self.claw_closed()
        if event.key() == Qt.Key_M:
            print("Base Shift Right")
            # self.base_right()
        if event.key() == Qt.Key_N:
            print("Bace Shift Left")
            # self.base_left()
        if event.key() == Qt.Key_U:
            print("Bottom Joint Foward")
            # self.bottom_joint_foward()
        if event.key() == Qt.Key_J:
            print("Bottom Joint Backwards")
            # self.bottom_joint_backwards()
        if event.key() == Qt.Key_I:
            print("Middle Joint Foward")
            # self.middle_joint_foward()
        if event.key() == Qt.Key_K:
            print("Middle Joint Backwards")
            # self.middle_joint_backwards()
        if event.key() == Qt.Key_O:
            print("Middle Joint Foward")
            # self.top_joint_foward()
        if event.key() == Qt.Key_L:
            print("Middle Joint Backwards")
            # self.top_joint_backwards()
        if event.key() == Qt.Key_Y:
            print("Wrist Clockwise")
            # self.wrist_clockwise()
        if event.key() == Qt.Key_H:
            print("Wrist Counterclockwise")
            # self.wrist_counterclockwise()
        if event.key() == Qt.Key_Escape:
            print("Stop All")
            # self.stop_all()

    def claw_open(self):
        serArm.write("Claw Open".encode())
    
    def claw_closed(self):
        serArm.write("Claw Closed".encode)
    
    def base_right(self):
        serArm.write("Base Shift Right".encode)
    
    def base_left(self):
        serArm.write("Base Shift Left".encode)
    
    def bottom_joint_foward(self):
        serArm.write("Bottom Joint Foward".encode)

    def bottom_joint_backwards(self):
        serArm.write("Bottom Joint Backwards".encode)

    def middle_joint_foward(self):
        serArm.write("Middle Joint Foward".encode)

    def middle_joint_backwards(self):
        serArm.write("Middle Joint Backwards".encode)

    def top_joint_foward(self):
        serArm.write("Top Joint Foward".encode)

    def top_joint_backwards(self):
        serArm.write("Top Joint Backwards".encode)
    
    def stop_all(self):
        serArm.write("Stop All".encode)

    def wrist_clockwise():
        serArm.write("Wrist Clockwise".encode)

    def wrist_counterclockwise():
        serArm.write("Wrist Counterclockwise".encode)
"""


if __name__ == "__main__":
    app = QApplication(sys.argv)

    main = MainWindow()
    main.show()

    sys.exit(app.exec())
