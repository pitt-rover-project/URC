On the local:

`./start.sh`

# For MacOS
`docker compose down`
*keep track of `ipconfig getifaddr en0`*
`export DISPLAY=192.168.x.x:0`  # Use Mac's local IP
`xhost + 192.168.x.x`
`docker-compose up --build`

`sudo docker exec -it pitt_urc_local bash`

`source /opt/ros/humble/local_setup.bash` in the docker container

`python3 -m guis.gen_gui`

---
__NOTES:__
Before launching the container, allow X11 connections from Docker:
For Linux -> `xhost +local:docker`
For MacOS -> `Open XQuartz → XQuartz → X11 Preferences → Security → Enable` and `Allow connections from network clients` -> `xhost + 127.0.0.1`
