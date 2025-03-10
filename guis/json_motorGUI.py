import serial
import sys
import requests
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import *
from PyQt5 import *

# Rover movement definitions
FORWARD = 1
REVERSE = 2
TURNLEFT = 3
TURNRIGHT = 4
BOOST = 5
SLOWDOWN = 6
STOP = 7
# arm movement definitions
CLAW_OPEN = 0
CLAW_CLOSE = 1
BS_RIGHT = 2
BS_LEFT = 3
BOTTOM_FORWARD = 4
BOTTOM_REVERSE = 5
MIDDLE_FORWARD = 6
MIDDLE_REVERSE = 7
TOP_FORWARD = 8
TOP_REVERSE = 9
WRIST_CLOCK = 10
WRIST_COUNTERCLOCK = 11
ALL_STOP = 12
# http server address
address = "http://localhost:8000/"

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.UI()

        self.window_2 = MotorWindow()
        self.window_3 = ArmWindow()

    def UI(self):
        self.title = QLabel('Choose between motor and arm.', self)    
        self.setWindowTitle("Motor and Arm Gui")

        buttonMotors = QPushButton('Motors',self)
        buttonArm = QPushButton('Arm', self)

        buttonMotors.clicked.connect(self.motor_on_click)
        buttonArm.clicked.connect(self.arm_on_click)

        self.setGeometry(1600,0,600,300)  

        
        self.title.setFont(QFont('Arial', 20))
        buttonMotors.resize(150, 50)
        buttonArm.resize(150,50)

        self.title.move(100,25)
        buttonMotors.move(50, 200)
        buttonArm.move(400, 200)

    def motor_on_click(self):
        print("motor clicked")
        self.window_2.show()
        
    def arm_on_click(self):
        print("arm clicked")
        self.window_3.show()

#commPort = '/dev/ttyACM0'
#serMotor = serial.Serial(commPort, baudrate = 115200)

class MotorWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.UI_Motor()

    def UI_Motor(self):
        self.setWindowTitle("Motor Control")
        self.setGeometry(1600,430,600,350)

        self.text = QLabel('Fowards = I || Backwards = , || Right = L || Left = J', self)  
        self.text2 = QLabel('Speed Up = Q || Slow Down = Z || Stop = K', self)
        self.text.setFont(QFont('Arial', 15))      
        self.text.move(60,50)
        self.text2.setFont(QFont('Arial', 15))      
        self.text2.move(150,150)

        closeButton = QPushButton('Close', self)
        closeButton.clicked.connect(self.close_on_click)
        closeButton.resize(150, 50)
        closeButton.move(225,250)

    def close_on_click(self):
        self.close()

    def keyPressEvent(self, event):
        if(event.key() == Qt.Key_I):
            print('Foward')
            self.foward()
        if(event.key() == Qt.Key_Comma):
            print("backwards")
            self.backward()     
        if(event.key() == Qt.Key_J):
            print('Left')
            self.right()
        if(event.key() == Qt.Key_L):
            print('Right')
            self.left()
        if(event.key() == Qt.Key_Q):
            print('Speed Up')
            self.speed_up()
        if(event.key() == Qt.Key_Z):
            print('Slow Down')
            self.slow_down()
        if(event.key() == Qt.Key_K):
            print('stop')
            self.stop()

    def foward(self):
        jsonData = {"rover": FORWARD}
        requests.post(address, json=jsonData)
    
    def backward(self):
        jsonData = {"rover": REVERSE}
        requests.post(address, json=jsonData)
    
    def left(self):
        jsonData = {"rover": TURNLEFT}
        requests.post(address, json=jsonData)
    
    def right(self):
        jsonData = {"rover": TURNRIGHT}
        requests.post(address, json=jsonData)
    
    def speed_up(self):
        jsonData = {"rover": BOOST}
        requests.post(address, json=jsonData)
    
    def slow_down(self):
        jsonData = {"rover": SLOWDOWN}
        requests.post(address, json=jsonData)

    def stop(self):
        jsonData = {"rover": STOP}
        requests.post(address, json=jsonData)

# commPort2 = '/dev/ttyACM1'
#serArm = serial.Serial(commPort2, baudrate = 115200)
class ArmWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.UI_Arm()

    def UI_Arm(self):
        self.setWindowTitle("Arm Control")
        self.setGeometry(1600,950,600,350)

        self.text = QLabel('Claw Opened = X || Claw Closed = C', self)  
        self.text2 = QLabel('Base Shift Right = D || Base Shift Left = A', self)
        self.text3 = QLabel('Forwards:', self)
        self.text4 = QLabel('Bottom Joint = U || Middle Joint = I || Top Joint = O', self)
        self.text5 = QLabel('Backwards: ', self)
        self.text6 = QLabel('Bottom Joint = J || Middle Joint = K || Top Joint = L', self)
        self.text7 = QLabel('Wrist Clockwise = Y || Wrist Counterclockwise = H', self)
        self.text8 = QLabel('Stop All = Escape', self)
        
        self.text.setFont(QFont('Arial', 15))      
        self.text.move(50,25)
        self.text2.setFont(QFont('Arial', 15))      
        self.text2.move(50,75)
        self.text3.setFont(QFont('Arial', 15))      
        self.text3.move(50,120)
        self.text4.setFont(QFont('Arial', 13))      
        self.text4.move(50,140)
        self.text5.setFont(QFont('Arial', 15))      
        self.text5.move(50,170)
        self.text6.setFont(QFont('Arial', 13))      
        self.text6.move(50,190)
        self.text7.setFont(QFont('Arial', 15))      
        self.text7.move(50,230)
        self.text8.setFont(QFont('Arial', 15))
        self.text8.move(50, 260)

        closeButton = QPushButton('Close', self)
        closeButton.clicked.connect(self.close_on_click)
        closeButton.resize(150, 50)
        closeButton.move(225,275)

    def close_on_click(self):
        self.close()
 
    def keyPressEvent(self, event):
        if(event.key() == Qt.Key_0):
            print('Claw Opened')
            self.claw_open()
        if(event.key() == Qt.Key_9):
            print("Claw Closed")
            self.claw_closed()     
        if(event.key() == Qt.Key_M):
            print('Base Shift Right')
            self.base_right()
        if(event.key() == Qt.Key_N):
            print('Bace Shift Left')
            self.base_left()
        if(event.key() == Qt.Key_U):
            print('Bottom Joint Foward')
            self.bottom_joint_foward()
        if(event.key() == Qt.Key_J):
            print('Bottom Joint Backwards')
            self.bottom_joint_backwards()
        if(event.key() == Qt.Key_I):
            print('Middle Joint Foward')
            self.middle_joint_foward()
        if(event.key() == Qt.Key_K):
            print('Middle Joint Backwards')
            self.middle_joint_backwards()
        if(event.key() == Qt.Key_O):
            print('Middle Joint Foward')
            self.top_joint_foward()
        if(event.key() == Qt.Key_L):
            print('Middle Joint Backwards')
            self.top_joint_backwards()
        if(event.key() == Qt.Key_Y):
            print('Wrist Clockwise')
            self.wrist_clockwise()
        if(event.key() == Qt.Key_H):
            print('Wrist Counterclockwise')
            self.wrist_counterclockwise()
        if(event.key() == Qt.Key_Escape):
            print('Stop All')
            self.stop_all()

    def claw_open(self):
        jsonData = {"arm": CLAW_OPEN}
        requests.post(address, json=jsonData)
    
    def claw_closed(self):
        jsonData = {"arm": CLAW_CLOSE}
        requests.post(address, json=jsonData)
    
    def base_right(self):
        jsonData = {"arm": BS_RIGHT}
        requests.post(address, json=jsonData)
    
    def base_left(self):
        jsonData = {"arm": BS_LEFT}
        requests.post(address, json=jsonData)
    
    def bottom_joint_foward(self):
        jsonData = {"arm": BOTTOM_FORWARD}
        requests.post(address, json=jsonData)

    def bottom_joint_backwards(self):
        jsonData = {"arm": BOTTOM_REVERSE}
        requests.post(address, json=jsonData)

    def middle_joint_foward(self):
        jsonData = {"arm": MIDDLE_FORWARD}
        requests.post(address, json=jsonData)

    def middle_joint_backwards(self):
        jsonData = {"arm": MIDDLE_REVERSE}
        requests.post(address, json=jsonData)

    def top_joint_foward(self):
        jsonData = {"arm": TOP_FORWARD}
        requests.post(address, json=jsonData)

    def top_joint_backwards(self):
        jsonData = {"arm": TOP_REVERSE}
        requests.post(address, json=jsonData)
    
    def stop_all(self):
        jsonData = {"arm": ALL_STOP}
        requests.post(address, json=jsonData)

    def wrist_clockwise():
        jsonData = {"arm": WRIST_CLOCK}
        requests.post(address, json=jsonData)

    def wrist_counterclockwise():
        jsonData = {"arm": WRIST_COUNTERCLOCK}
        requests.post(address, json=jsonData)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    main = MainWindow()
    main.show()

    sys.exit(app.exec())