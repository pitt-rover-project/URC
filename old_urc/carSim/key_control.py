#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import sys, select, termios, tty

# Map for movement keys
move_bindings = {
    'i': [1, 0, 0, 0],
    'o': [1, 0, 0, -1],
    'j': [0, 0, 0, 1],
    'l': [0, 0, 0, -1],
    'u': [1, 0, 0, 1],
    ',': [-1, 0, 0, 0],
    '.': [-1, 0, 0, 1],
    'm': [-1, 0, 0, -1],
    'O': [1, -1, 0, 0],
    'I': [1, 0, 0, 0],
    'J': [0, 1, 0, 0],
    'L': [0, -1, 0, 0],
    'U': [1, 1, 0, 0],
    '<': [-1, 0, 0, 0],
    '>': [-1, -1, 0, 0],
    'M': [-1, 1, 0, 0],
    'k': [0, 0, 0, 0],
    'K': [0, 0, 0, 0]
}

# Map for speed keys
speed_bindings = {
    'q': [1.1, 1.1],
    'z': [0.9, 0.9],
    'w': [1.1, 1],
    'x': [0.9, 1],
    'e': [1, 1.1],
    'c': [1, 0.9]
}

# Reminder message
msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For strafing, hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

# Init variables
speed = 0.5  # Linear velocity (m/s)
turn = 1.0   # Angular velocity (rad/s)
x, y, th = 0, 0, 0  # Forward/backward/neutral direction vars
key = ' '

# For non-blocking keyboard inputs
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def teleop_twist_keyboard():
    global speed, turn, x, y, th, key

    # Init ROS node
    rospy.init_node('teleop_twist_keyboard')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twist = Twist()

    print(msg)
    print("\rCurrent: speed {}\tturn {} | Awaiting command...\r".format(speed, turn))

    while True:
        # Get the pressed key
        key = getch()

        # If the key corresponds to a key in move_bindings
        if key in move_bindings:
            # Grab the direction data
            x, y, _, th = move_bindings[key]
            print("\rCurrent: speed {}\tturn {} | Last command: {}   ".format(speed, turn, key))

        # Otherwise if it corresponds to a key in speed_bindings
        elif key in speed_bindings:
            # Grab the speed data
            speed *= speed_bindings[key][0]
            turn *= speed_bindings[key][1]
            print("\rCurrent: speed {}\tturn {} | Last command: {}   ".format(speed, turn, key))

        # Otherwise, set the robot to stop
        else:
            x, y, th = 0, 0, 0

            # If ctrl-C (^C) was pressed, terminate the program
            if key == '\x03':
                break

            print("\rCurrent: speed {}\tturn {} | Invalid command! {}".format(speed, turn, key))

        # Update the Twist message
        twist.linear.x = x * speed
        twist.linear.y = y * speed
        twist.angular.z = th * turn

        # Publish it
        pub.publish(twist)
        rospy.sleep(0.1)  # Add a small delay to control the rate

if __name__ == '__main__':
    try:
        teleop_twist_keyboard()
    except rospy.ROSInterruptException:
        pass
