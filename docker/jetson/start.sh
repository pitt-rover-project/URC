#!/bin/bash

sudo docker build -t jetson_bridge .
sudo docker run -d --rm -it --runtime nvidia --network host --gpus all --device=/dev/ttyACM0 -e DISPLAY --name pitt_urc_jetson jetson_bridge