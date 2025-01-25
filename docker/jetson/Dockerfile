FROM ubuntu:22.04

WORKDIR /app

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && apt-get upgrade -y && apt-get install locales -y &&\
    locale-gen en_US en_US.UTF-8 &&\
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 &&\
    export LANG=en_US.UTF-8

RUN apt-get install software-properties-common -y &&\
    add-apt-repository universe


RUN apt-get update && apt-get install -y tzdata

RUN ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && dpkg-reconfigure -f noninteractive tzdata

RUN apt-get update -y && apt install curl -y &&\
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install ros-humble-desktop -y

RUN apt-get install ros-dev-tools -y

RUN apt-get update -y && apt-get install -y python3-pip &&\
    pip install supervisor &&\
    pip3 install --no-cache-dir pyserial

# https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE &&\
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u &&\
    apt-get install -y librealsense2-utils &&\
    apt-get install -y librealsense2-dev

RUN apt-get install ros-humble-librealsense2* -y

RUN apt-get install ros-humble-realsense2-* -y

RUN git clone https://github.com/pitt-rover-project/URC.git /app

RUN mkdir -p /etc/supervisor/conf.d/

RUN cp /app/docker/jetson/supervisord.conf /etc/supervisor/conf.d/supervisord.conf

CMD ["supervisord", "-n", "-c", "/etc/supervisor/conf.d/supervisord.conf"]
