#!/bin/bash

# Builds the Dockerfile in this directory
sudo docker build -t urc_local .

# Sets the DISPLAY environment variable to allow GUI applications to connect
# This assumes you are running on macOS and have XQuartz installed
# For MacOS -> `Open XQuartz → XQuartz → X11 Preferences → Security → Enable` 
# and `Allow connections from network clients` -> `xhost + $DISPLAY`
export DISPLAY=$(ipconfig getifaddr en0):0

# Allow connections from the local machine to the X server
# This is necessary for the GUI to be visible from within the Docker container (X11 forwarding to MacOS)
xhost +$(ipconfig getifaddr en0)

# Launches the docker container in detached mode using compose (docker-compose.yml)
sudo docker-compose up -d

# Wait for the container to be fully up and running (not really needed, just for "safety")
sleep 10

# Opens a bash shell in the running Docker container (contains all rover code)
sudo docker exec -it pitt_urc_local bash

# Echo (prints) successfully inside the container message
echo "Successfully inside the container."
