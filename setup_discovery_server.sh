#!/bin/bash
echo "🚀 Setting up ROS2 Discovery Server Solution"
echo "============================================"

# Get IP addresses
JETSON_IP="$1"
MAC_HOST_IP="$2"

if [ -z "$JETSON_IP" ] || [ -z "$MAC_HOST_IP" ]; then
    echo "❌ Usage: $0 <jetson_ip> <mac_host_wifi_ip>"
    echo "   Example: $0 192.168.1.91 192.168.1.100"
    echo ""
    echo "🚨 IMPORTANT: Use WIFI IPs, not Docker container IPs!"
    echo "   Jetson IP: Check with 'hostname -I' on Jetson"
    echo "   Mac Host IP: Your Mac's WiFi IP (not 192.168.65.x)"
    exit 1
fi

echo "📝 Configuring Discovery Server..."
echo "   Jetson IP: $JETSON_IP"
echo "   Mac Host WiFi IP: $MAC_HOST_IP"

# Create discovery server script for MacBook (server)
cat > docker/local/start_discovery_server.sh << EOF
#!/bin/bash
echo "🏃 Starting Discovery Server on MacBook..."
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="$MAC_HOST_IP:11888"

# Start discovery server
ros2 run fastdds discovery fastdiscoveryserver -i 0 -l 0.0.0.0 -p 11888 &
DISCOVERY_PID=\$!

echo "✅ Discovery Server running on $MAC_HOST_IP:11888 (PID: \$DISCOVERY_PID)"
echo "   Kill with: kill \$DISCOVERY_PID"
wait
EOF

# Create client setup script for containers
cat > setup_ros2_client.sh << EOF
#!/bin/bash
echo "🔗 Configuring ROS2 Client for Discovery Server..."

# Set environment variables
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# For MacBook container (connects to local server)
if [ "\$HOSTNAME" = "docker-desktop" ]; then
    export ROS_DISCOVERY_SERVER="$MAC_HOST_IP:11888"
    echo "✅ MacBook container configured for Discovery Server"
else
    # For Jetson container (connects to remote server)
    export ROS_DISCOVERY_SERVER="$MAC_HOST_IP:11888"
    echo "✅ Jetson container configured for Discovery Server"
fi

echo "🚀 Ready to test! Run: ros2 run demo_nodes_cpp talker"
echo "                      ros2 run demo_nodes_cpp listener"
EOF

chmod +x docker/local/start_discovery_server.sh
chmod +x setup_ros2_client.sh

echo ""
echo "📋 SETUP COMPLETE! Follow these steps:"
echo ""
echo "1. ON MACBOOK: Start Discovery Server"
echo "   cd docker/local && ./start_discovery_server.sh"
echo ""
echo "2. IN EACH CONTAINER: Configure client"
echo "   source /app/setup_ros2_client.sh"
echo ""
echo "3. TEST COMMUNICATION:"
echo "   MacBook container: ros2 run demo_nodes_cpp talker"
echo "   Jetson container:  ros2 run demo_nodes_cpp listener"
echo ""
echo "✨ This bypasses multicast issues entirely!"