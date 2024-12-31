FROM dustynv/ros:humble-ros-base-l4t-r32.7.1

WORKDIR /app

RUN git clone https://github.com/pitt-rover-project/URC.git /app

RUN apt-get update && apt-get install -y python3-pip && \
    pip install supervisor &&\
    pip3 install --no-cache-dir pyserial

RUN cp /app/docker/jetson/supervisord.conf /etc/supervisor/conf.d/supervisord.conf

CMD ["supervisord", "-n", "-c", "/etc/supervisor/conf.d/supervisord.conf"]
