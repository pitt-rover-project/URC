## Running ROS2 in a Specialized Ubuntu Environment - Docker: Local Machine

### On your local machine, run one of the following commands:
- `./start_mac.sh` _(for MacOS)_
- `./start_ubuntu.sh` _(for Linux and (maybe?) Windows)_

<p> This shell script will build a Docker image based on the Dockerfile provided. After the build stage is complete, the shell script boots up an X server that will allow the gui application to be forwarded to the local machine (it will make the GUI launched in the Ubuntu Docker container visible in the machine used to launch the Docker container (MacOS, Linux, Windows)). Afterwards, the image is "booted" following the specifications in the docker-compose file and one enters the shell of the container. </p> 

### Once Inside the Shell of the Docker Container, run the following command:
- `source /opt/ros/humble/local_setup.bash`
#### NOTE: This must be done __*everytime*__ a new shell of the container is opened.

### From the main directory `/app`, run the following command (which launches the general_gui):
- `python3 -m guis.gen_gui`
