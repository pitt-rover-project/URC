# Run this script when start_<operating system>.sh ran successfully
# This means you should be inside the shell of the Docker Container (`root@docker-desktop:/app#`)

# Sources ROS2
source /opt/ros/humble/local_setup.bash

# Runs the GUI
ls && python3 -m guis.gen_gui
