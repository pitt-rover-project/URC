import subprocess

process = subprocess.Popen(["ros2", "run", "realsense2_camera", "realsense2_camera_node"])

process.wait()
