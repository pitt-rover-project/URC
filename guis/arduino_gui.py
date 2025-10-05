import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QGroupBox, QSlider, QFrame
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPalette, QColor
from guis.publishers.publisher import MotorPublisher
import rclpy
from rclpy.executors import SingleThreadedExecutor
import math

# Throttle control class
class ThrottleControl(QFrame):
    def __init__(self):
        super().__init__()
        self.throttle_value = 0
        self.setup_ui()
        
    def setup_ui(self):
        self.setObjectName("throttleFrame")
        layout = QVBoxLayout(self)
        
        # Title
        title = QLabel("THROTTLE CONTROL")
        title.setObjectName("throttleTitle")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Percentage display
        self.percentage_label = QLabel("0%")
        self.percentage_label.setObjectName("throttlePercentage")
        self.percentage_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.percentage_label)
        
        # Horizontal slider
        slider_frame = QFrame()
        slider_layout = QHBoxLayout(slider_frame)
        
        # Min/Max labels
        min_label = QLabel("0%")
        min_label.setObjectName("throttleMinMax")
        max_label = QLabel("100%")
        max_label.setObjectName("throttleMinMax")
        
        # Throttle slider
        self.throttle_slider = QSlider(Qt.Horizontal)
        self.throttle_slider.setObjectName("throttleSlider")
        self.throttle_slider.setRange(0, 100)
        self.throttle_slider.setValue(0)
        self.throttle_slider.valueChanged.connect(self.on_throttle_change)
        
        slider_layout.addWidget(min_label)
        slider_layout.addWidget(self.throttle_slider)
        slider_layout.addWidget(max_label)
        
        layout.addWidget(slider_frame)
        
        # Control buttons
        button_layout = QHBoxLayout()
        reset_btn = QPushButton("RESET")
        reset_btn.setObjectName("throttleButton")
        reset_btn.clicked.connect(self.reset_throttle)
        
        full_btn = QPushButton("FULL")
        full_btn.setObjectName("throttleButton")
        full_btn.clicked.connect(self.full_throttle)
        
        button_layout.addWidget(reset_btn)
        button_layout.addWidget(full_btn)
        layout.addLayout(button_layout)
        
    def on_throttle_change(self, value):
        self.throttle_value = value
        self.percentage_label.setText(f"{value}%")
        
    def reset_throttle(self):
        self.throttle_slider.setValue(0)
        
    def full_throttle(self):
        self.throttle_slider.setValue(100)
        
    def get_throttle_value(self):
        return self.throttle_value / 100.0  # Return as 0-1 value

class MainWindow(QWidget):
    def __init__(self, motor_publisher):
        super().__init__()
        # Movement state
        self.linear_velocity = 0.0  # 0 to 1
        self.angular_velocity = 0.0  # -1 to 1 (negative = left, positive = right)
        
        # Key states
        self.keys_pressed = set()
        self.shift_pressed = False
        
        # Acceleration and deceleration rates
        self.acceleration_rate = 0.02  # How fast it accelerates toward target speed
        self.deceleration_rate = 0.015
        self.angular_deceleration_rate = 0.1
        
        # Store motor publisher reference (passed from main)
        self.motor_publisher = motor_publisher
        
        self.setup_ui()
        self.apply_styles()
        
        # Update timer for physics simulation
        self.physics_timer = QTimer()
        self.physics_timer.timeout.connect(self.update_physics)
        self.physics_timer.start(50)  # 20 FPS update rate
        
        # Display timer for terminal output
        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self.display_values)
        self.display_timer.start(100)  # Update display every 100ms
    
    def publish_motor_command(self):
        """Publish motor command with linear velocity at position 0 and angular velocity at position 3"""
        if self.motor_publisher is not None:
            # Create motor values array: [linear, 0, 0, angular, 0, 0]
            motor_values = [
                self.linear_velocity,  # Position 0 (1st value): Linear velocity
                0,                      # Position 1 (2nd value): Unused
                0,                      # Position 2 (3rd value): Unused
                self.angular_velocity,  # Position 3 (4th value): Angular velocity
                0,                      # Position 4 (5th value): Unused
                0                       # Position 5 (6th value): Unused
            ]
            self.motor_publisher.publish_motor_command(motor_values)
        
    def setup_ui(self):
        # Get screen dimensions
        screen = QApplication.primaryScreen().availableGeometry()
        screen_width = screen.width()
        screen_height = screen.height()
        
        # Window settings
        window_width = int(screen_width * 0.7)
        window_height = int(screen_height * 0.6)
        x_offset = int((screen_width - window_width) / 2)
        y_offset = int((screen_height - window_height) / 2)
        
        self.setGeometry(x_offset, y_offset, window_width, window_height)
        self.setWindowTitle("Rover Control Interface")
        
        # Main layout
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)
        
        # Title
        title = QLabel("CAR CONTROL INTERFACE")
        title.setObjectName("mainTitle")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)
        
        # Main content layout
        content_layout = QHBoxLayout()
        
        # Left side - Motor controls
        motor_group = QGroupBox("Vehicle Controls")
        motor_group.setObjectName("controlGroup")
        motor_layout = QVBoxLayout(motor_group)
        
        # Instructions
        instructions = QLabel(
            "Controls:\n"
            "• W - Forward\n"
            "• S - Backward\n"
            "• A - Turn Left\n"
            "• D - Turn Right\n"
            "• SHIFT - Apply Throttle\n"
            "• Release SHIFT to decelerate"
        )
        instructions.setObjectName("instructions")
        motor_layout.addWidget(instructions)
        
        # Movement buttons grid
        button_grid = QGridLayout()
        
        self.forward_button = QPushButton("Forward\n(W)")
        self.forward_button.setObjectName("motorButton")
        
        self.left_button = QPushButton("Turn Left\n(A)")
        self.left_button.setObjectName("motorButton")
        
        self.backward_button = QPushButton("Backward\n(S)")
        self.backward_button.setObjectName("motorButton")
        
        self.right_button = QPushButton("Turn Right\n(D)")
        self.right_button.setObjectName("motorButton")
        
        self.brake_button = QPushButton("BRAKE\n(SPACE)")
        self.brake_button.setObjectName("stopButton")
        
        # Layout movement buttons
        button_grid.addWidget(self.forward_button, 0, 1)
        button_grid.addWidget(self.left_button, 1, 0)
        button_grid.addWidget(self.brake_button, 1, 1)
        button_grid.addWidget(self.right_button, 1, 2)
        button_grid.addWidget(self.backward_button, 2, 1)
        
        motor_layout.addLayout(button_grid)
        
        # Status display
        status_group = QGroupBox("Vehicle Status")
        status_group.setObjectName("statusGroup")
        status_layout = QVBoxLayout(status_group)
        
        self.linear_label = QLabel("Linear Velocity: 0.00")
        self.linear_label.setObjectName("statusLabel")
        self.angular_label = QLabel("Angular Velocity: 0.00")
        self.angular_label.setObjectName("statusLabel")
        self.throttle_status = QLabel("Throttle: OFF")
        self.throttle_status.setObjectName("throttleStatus")
        self.ros_status = QLabel("ROS2: Connected" if self.motor_publisher else "ROS2: Disconnected")
        self.ros_status.setObjectName("rosStatus")
        
        status_layout.addWidget(self.linear_label)
        status_layout.addWidget(self.angular_label)
        status_layout.addWidget(self.throttle_status)
        status_layout.addWidget(self.ros_status)
        
        motor_layout.addWidget(status_group)
        motor_layout.addStretch()
        
        # Right side - Throttle control
        self.throttle_control = ThrottleControl()
        
        # Add to content layout
        content_layout.addWidget(motor_group, 2)
        content_layout.addWidget(self.throttle_control, 1)
        
        main_layout.addLayout(content_layout)
        
    def apply_styles(self):
        style = """
        QWidget {
            background-color: #1e1e1e;
            color: #e0e0e0;
            font-family: 'Segoe UI', Arial, sans-serif;
            font-size: 12px;
        }
        
        #mainTitle {
            color: #ffffff;
            font-size: 24px;
            font-weight: bold;
            padding: 10px;
            margin-bottom: 10px;
        }
        
        QGroupBox {
            font-size: 14px;
            font-weight: bold;
            color: #ffffff;
            border: 2px solid #444444;
            border-radius: 8px;
            margin-top: 10px;
            padding-top: 15px;
            background-color: #2a2a2a;
        }
        
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 15px;
            padding: 0 10px 0 10px;
            background-color: #2a2a2a;
        }
        
        #controlGroup {
            border: 2px solid #42a5f5;
        }
        
        #controlGroup::title {
            color: #42a5f5;
        }
        
        #statusGroup {
            border: 2px solid #ff7043;
        }
        
        #statusGroup::title {
            color: #ff7043;
        }
        
        #instructions {
            color: #e0e0e0;
            font-size: 11px;
            padding: 10px;
            background-color: #2a2a2a;
            border: 1px solid #444444;
            border-radius: 4px;
            margin-bottom: 10px;
        }
        
        #motorButton {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #42a5f5, stop: 1 #1e88e5);
            border: none;
            border-radius: 6px;
            color: white;
            font-weight: bold;
            padding: 15px;
            margin: 3px;
            min-height: 60px;
        }
        
        #motorButton:hover {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #64b5f6, stop: 1 #42a5f5);
        }
        
        #motorButton:pressed {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #1e88e5, stop: 1 #1565c0);
        }
        
        #stopButton {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #f44336, stop: 1 #d32f2f);
            border: none;
            border-radius: 6px;
            color: white;
            font-weight: bold;
            padding: 15px;
            margin: 3px;
            min-height: 60px;
        }
        
        #stopButton:hover {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #e57373, stop: 1 #f44336);
        }
        
        #statusLabel {
            color: #66bb6a;
            font-size: 14px;
            font-weight: bold;
            background-color: #1a1a1a;
            border: 1px solid #444444;
            border-radius: 4px;
            padding: 8px;
            margin: 2px;
        }
        
        #throttleStatus {
            color: #ff7043;
            font-size: 14px;
            font-weight: bold;
            background-color: #1a1a1a;
            border: 1px solid #444444;
            border-radius: 4px;
            padding: 8px;
            margin: 2px;
        }
        
        #rosStatus {
            color: #42a5f5;
            font-size: 14px;
            font-weight: bold;
            background-color: #1a1a1a;
            border: 1px solid #444444;
            border-radius: 4px;
            padding: 8px;
            margin: 2px;
        }
        
        /* Throttle Control Styles */
        #throttleFrame {
            background-color: #2a2a2a;
            border: 2px solid #444444;
            border-radius: 10px;
            padding: 15px;
            margin: 5px;
        }
        
        #throttleTitle {
            color: #ffffff;
            font-size: 16px;
            font-weight: bold;
            margin-bottom: 10px;
        }
        
        #throttlePercentage {
            color: #42a5f5;
            font-size: 36px;
            font-weight: bold;
            margin: 10px;
        }
        
        #throttleMinMax {
            color: #e0e0e0;
            font-size: 10px;
        }
        
        #throttleSlider {
            background-color: #1a1a1a;
            border-radius: 4px;
            height: 20px;
        }
        
        #throttleSlider::groove:horizontal {
            background: #1a1a1a;
            border: 1px solid #555555;
            height: 20px;
            border-radius: 4px;
        }
        
        #throttleSlider::handle:horizontal {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #42a5f5, stop: 1 #1e88e5);
            border: 2px solid #42a5f5;
            width: 30px;
            border-radius: 10px;
            margin: -5px 0;
        }
        
        #throttleSlider::handle:horizontal:hover {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #64b5f6, stop: 1 #42a5f5);
            border: 2px solid #64b5f6;
        }
        
        #throttleButton {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #42a5f5, stop: 1 #1e88e5);
            border: none;
            border-radius: 4px;
            color: white;
            font-weight: bold;
            padding: 8px;
            margin: 3px;
            min-height: 30px;
        }
        
        #throttleButton:hover {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                        stop: 0 #64b5f6, stop: 1 #42a5f5);
        }
        """
        self.setStyleSheet(style)
    
    def update_physics(self):
        """Update physics simulation for car movement"""
        throttle = self.throttle_control.get_throttle_value()
        
        # Check if shift is pressed for throttle application
        if self.shift_pressed and (Qt.Key_W in self.keys_pressed or 
                                  Qt.Key_S in self.keys_pressed or
                                  Qt.Key_A in self.keys_pressed or 
                                  Qt.Key_D in self.keys_pressed):
            # Apply throttle - cap linear velocity at throttle level
            if Qt.Key_W in self.keys_pressed:
                target_linear = throttle  # Forward capped by throttle
                self.linear_velocity = min(target_linear, self.linear_velocity + self.acceleration_rate)
            elif Qt.Key_S in self.keys_pressed:
                target_linear = -throttle  # Backward capped by throttle
                self.linear_velocity = max(target_linear, self.linear_velocity - self.acceleration_rate)
            
            # Angular velocity for turning
            if Qt.Key_A in self.keys_pressed:
                target_angular = -throttle  # Negative for left
                self.angular_velocity += (target_angular - self.angular_velocity) * 0.1
            elif Qt.Key_D in self.keys_pressed:
                target_angular = throttle  # Positive for right
                self.angular_velocity += (target_angular - self.angular_velocity) * 0.1
            else:
                # Return angular to center when not turning
                self.angular_velocity *= 0.8
                
        else:
            # Decelerate when shift is not pressed
            if abs(self.linear_velocity) > 0.001:
                if self.linear_velocity > 0:
                    self.linear_velocity = max(0, self.linear_velocity - self.deceleration_rate)
                else:
                    self.linear_velocity = min(0, self.linear_velocity + self.deceleration_rate)
            else:
                self.linear_velocity = 0
            
            # Decelerate angular velocity
            if abs(self.angular_velocity) > 0.001:
                self.angular_velocity *= 0.7
            else:
                self.angular_velocity = 0
        
        # Publish motor command to ROS2
        self.publish_motor_command()
        
        # Update display
        self.linear_label.setText(f"Linear Velocity: {self.linear_velocity:.2f}")
        self.angular_label.setText(f"Angular Velocity: {self.angular_velocity:.2f}")
        self.throttle_status.setText(f"Throttle: {'ON' if self.shift_pressed else 'OFF'}")
        
        # Update button visual states
        self.update_button_states()
    
    def display_values(self):
        """Output current values to terminal"""
        print(f"Linear: {self.linear_velocity:.3f} | Angular: {self.angular_velocity:.3f} | Throttle: {self.throttle_control.get_throttle_value():.2f}")
    
    def update_button_states(self):
        """Update visual feedback on buttons"""
        # Reset all buttons to default style
        buttons = {
            Qt.Key_W: self.forward_button,
            Qt.Key_A: self.left_button,
            Qt.Key_S: self.backward_button,
            Qt.Key_D: self.right_button,
        }
        
        for key, button in buttons.items():
            if key in self.keys_pressed and self.shift_pressed:
                button.setStyleSheet("""
                    background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                              stop: 0 #00ff00, stop: 1 #008800);
                    border: 2px solid #00ff00;
                    border-radius: 6px;
                    color: black;
                    font-weight: bold;
                    padding: 15px;
                    margin: 3px;
                    min-height: 60px;
                """)
            else:
                button.setStyleSheet("")  # Reset to default from main stylesheet
    
    def keyPressEvent(self, event):
        """Handle key press events"""
        if event.key() == Qt.Key_Shift:
            self.shift_pressed = True
        elif event.key() == Qt.Key_Space:
            # Emergency brake
            self.linear_velocity = 0
            self.angular_velocity = 0
            self.publish_motor_command()
        elif event.key() in [Qt.Key_W, Qt.Key_A, Qt.Key_S, Qt.Key_D]:
            self.keys_pressed.add(event.key())
        
        # Prevent key repeat
        if not event.isAutoRepeat():
            self.keys_pressed.add(event.key())
    
    def keyReleaseEvent(self, event):
        """Handle key release events"""
        if event.key() == Qt.Key_Shift:
            self.shift_pressed = False
        
        # Prevent key repeat
        if not event.isAutoRepeat():
            self.keys_pressed.discard(event.key())

if __name__ == "__main__":
    # Initialize ROS2
    rclpy.init()
    
    # Create QApplication
    app = QApplication(sys.argv)
    
    # Create motor publisher (similar to how IMU subscriber is created in main GUI)
    motor_pub = MotorPublisher(
        topic_name='motor_control_input',
        node_name='rover_control_gui'
    )
    
    # Create main window and pass the publisher
    main = MainWindow(motor_pub)
    main.setFocusPolicy(Qt.StrongFocus)
    main.show()
    
    # Use a ROS executor that doesn't block the Qt event loop
    executor = SingleThreadedExecutor()
    executor.add_node(motor_pub)
    
    # Use a QTimer to periodically spin ROS events (same pattern as main GUI)
    timer = QTimer()
    timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0))
    timer.start(10)  # 10ms interval
    
    # Run Qt application
    exit_code = app.exec_()
    
    # Cleanup
    motor_pub.stop_all_motors()
    executor.shutdown()
    motor_pub.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)