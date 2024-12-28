#! /bin/bash

# TODO : change the file names to the actual repos
sudo docker exec -it pitt_urc_local ros2 launch urdf_tutorial display.launch.py model:=/home/ed/my_robot.urdf
