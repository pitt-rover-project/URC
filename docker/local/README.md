# ROS 2 Humble in Docker (Local Machine)

A Dockerized Ubuntu environment pre-configured with ROS 2 Humble, with X11 forwarding enabled for GUI applications.

## Prerequisites
- Docker & Docker Compose installed
- For macOS: XQuartz (enable "Allow connections from network clients")
- For Linux: an X server (ensure `x11-xserver-utils` or equivalent is installed)
- Permissions to run `xhost`

## Inside the Local Machine
### Docker Container Initialization
```bash
# macOS
./start_mac.sh

# Linux
./start_linux.sh
```

These scripts build the Docker image, configure X11 forwarding, start the containers via Docker Compose, and open an interactive shell inside the container.

## Inside the Container
### Sourcing ROS 2 Humble Setup Script
This step configures your shell to use the ROS 2 Humble installation within the container by sourcing the setup script, setting up all necessary environment variables and paths for ROS 2 commands.

```bash
source /opt/ros/humble/local_setup.bash
```

> **Note:** You must run this in every new shell session inside the container.

### Launching the GUI 
```bash
cd /app                 # Should be here by default!
python3 -m guis.gen_gui # Starts the General Gui
```
