#!/bin/bash

## TODO: THIS NEEDS TO BE TESTED ON A LINUX MACHINE WITH DOCKER INSTALLED ##

# Builds the Dockerfile in this directory
sudo docker build -t urc_local .

# Allows GUI applications to connect to the host's X server
# This assumes you are running on Linux and have an X server running
xhost +local:docker

# Launches the docker container in detached mode using compose (docker-compose.yml)
sudo docker compose up -d

# Wait for the container to be fully up and running (not really needed, just for "safety")
sleep 10

# Opens a bash shell in the running Docker container (contains all rover code)
sudo docker exec -it pitt_urc_local bash

# Echo (prints) successfully inside the container message
echo "Successfully inside the container."
