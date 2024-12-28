FROM dustynv/ros:humble-ros-base-l4t-r32.7.1

WORKDIR /app

RUN git clone https://github.com/pitt-rover-project/URC.git /app

RUN apt-get update && apt-get install -y python3-pip

RUN apt-get install ros-humble-urdf-tutorial
