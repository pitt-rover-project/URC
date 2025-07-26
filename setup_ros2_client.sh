#!/bin/bash
echo "🔗 Configuring ROS2 Client for Discovery Server..."

# Set environment variables
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# For MacBook container (connects to local server)
if [ "$HOSTNAME" = "docker-desktop" ]; then
    export ROS_DISCOVERY_SERVER="192.168.1.73:11888"
    echo "✅ MacBook container configured for Discovery Server"
else
    # For Jetson container (connects to remote server)
    export ROS_DISCOVERY_SERVER="192.168.1.73:11888"
    echo "✅ Jetson container configured for Discovery Server"
fi

echo "🚀 Ready to test! Run: ros2 run demo_nodes_cpp talker"
echo "                      ros2 run demo_nodes_cpp listener"
