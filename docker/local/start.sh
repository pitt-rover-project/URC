#!/bin/bash

sudo docker build -t urc_local .
sudo docker run -d --rm -it --name pitt_urc_local urc_local