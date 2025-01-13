FROM dustynv/ros:humble-ros-base-l4t-r32.7.1

WORKDIR /app

RUN git clone https://github.com/pitt-rover-project/URC.git /app

RUN apt-get update && apt-get install -y python3-pip && \
    pip install supervisor &&\
    pip3 install --no-cache-dir pyserial

# https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get install -y librealsense2-utils
RUN apt-get install -y librealsense2-dev

RUN mkdir -p /etc/supervisor/conf.d/

RUN cp /app/docker/jetson/supervisord.conf /etc/supervisor/conf.d/supervisord.conf

CMD ["supervisord", "-n", "-c", "/etc/supervisor/conf.d/supervisord.conf"]
