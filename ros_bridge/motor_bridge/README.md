# Unified Motor Control System

## Overview

This directory contains the unified motor control system for the University of Pittsburgh rover project. The system has been streamlined to follow KISS (Keep It Stupid Simple) principles and provides a single point of control for all motor operations.

## Architecture

### Simplified Data Flow
```
GUI Publishers → motor_control_input (String) →
Teleop Systems → cmd_vel (Twist)            → MotorBridge → Arduino
Legacy Systems → drive_arduino_commands     →
```

### Key Components

1. **MotorBridge (Enhanced)**: Located in `../arduino_bridge_base/arduino_bridge_base.py`
   - Handles Twist messages from `cmd_vel` topic
   - Handles String messages from `motor_control_input` topic
   - Handles legacy String messages from `drive_arduino_commands` topic
   - Single serial connection to Arduino
   - Unified error handling and logging

2. **Arduino Code**: Located in `../motor_subscriber/motor_serial/motor_serial.ino`
   - Simplified parsing using `sscanf()` instead of manual string manipulation
   - Removed extensive commented-out dead code
   - Clean, focused implementation

3. **GUI Publishers**: Located in `../guis/publishers/publisher.py`
   - MotorPublisher class for GUI integration
   - TwistPublisher class for velocity commands
   - PyQt5 integration with signals and slots

## Usage

### Starting the Motor Bridge
```bash
# From this directory
python3 launch_motor_bridge.py

# Or from the project root
python3 ros_bridge/motor_bridge/launch_motor_bridge.py
```

### Sending Motor Commands

#### From GUI Applications
```python
from guis.publishers.publisher import MotorPublisher

motor_pub = MotorPublisher()
motor_pub.publish_motor_command([10, 20, 30, 40, 50, 60])  # 6 motor values
motor_pub.stop_all_motors()  # Emergency stop
```

#### From ROS2 Command Line
```bash
# String format (GUI style)
ros2 topic pub /motor_control_input std_msgs/String "data: '1,2,3,4,5,6'"

# Twist format (teleop style)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
```

## Message Formats

### String Commands (motor_control_input)
- Format: `"value1,value2,value3,value4,value5,value6"`
- Values: Float numbers representing motor speeds/positions
- Example: `"0,0,0,0,0,0"` (stop all motors)

### Twist Commands (cmd_vel)
- Standard ROS2 geometry_msgs/Twist format
- Automatically converted to 6-value comma-separated format
- Sent as: `linear.x,linear.y,linear.z,angular.x,angular.y,angular.z`

## Improvements Made

### Arduino Code Simplification
- ✅ Removed duplicate #include statements
- ✅ Removed 125+ lines of commented-out dead code
- ✅ Replaced complex manual parsing with simple `sscanf()`
- ✅ Cleaned up debug output
- ✅ Removed redundant function declarations

### Python Code Unification
- ✅ Enhanced MotorBridge to accept multiple input types
- ✅ Added proper validation for String commands
- ✅ Unified error handling and logging
- ✅ Standardized topic names

### System Architecture
- ✅ Single point of Arduino communication (no conflicts)
- ✅ Multiple input methods converge to one output
- ✅ Consistent message validation
- ✅ Clear data flow documentation

## Topic Reference

| Topic | Message Type | Purpose | Publisher | Subscriber |
|-------|--------------|---------|-----------|------------|
| `cmd_vel` | Twist | Teleop control | External teleop | MotorBridge |
| `motor_control_input` | String | GUI control | MotorPublisher | MotorBridge |
| `drive_arduino_commands` | String | Legacy control | Various | MotorBridge |
| `drive_data` | String | Arduino responses | MotorBridge | Various |

## Migration Notes

### From Old motor_subscriber System
The previous `motor_subscriber` system has been **replaced** by the enhanced MotorBridge.

**Old system** (deprecated):
```
GUI → motor_control_input → MotorSubscriber → motor_commands → [NO CONNECTION]
```

**New system** (current):
```
GUI → motor_control_input → MotorBridge → Arduino
```

### Backward Compatibility
- All existing topic names are preserved
- Legacy `drive_arduino_commands` topic still supported
- No changes required to existing GUI code
- Arduino receives the same message format

## Testing

### Verify Motor Bridge is Running
```bash
ros2 node list | grep motor_bridge
```

### Check Topic Connections
```bash
ros2 topic list | grep -E "(cmd_vel|motor_control|drive_)"
ros2 topic info /motor_control_input
```

### Monitor Arduino Communication
```bash
ros2 topic echo /drive_data
```

## Troubleshooting

### Serial Port Issues
- Ensure `/dev/ttyACM0` is available
- Check Arduino connection and baud rate (115200)
- Verify user permissions for serial access

### Topic Not Found
- Confirm MotorBridge is running: `ros2 node list`
- Check topic names: `ros2 topic list`
- Verify ROS2 environment is sourced

### No Motor Response
- Check Arduino serial monitor for incoming data
- Verify message format is correct (6 comma-separated values)
- Ensure motor hardware connections are secure

This unified system provides a clean, maintainable foundation for rover motor control while preserving all existing functionality.