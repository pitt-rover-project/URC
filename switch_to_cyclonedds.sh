#!/bin/bash
echo "🔄 Switching to CycloneDDS for Better Multi-Machine Support"
echo "=========================================================="

# Get IP addresses
JETSON_IP="$1"
MACBOOK_IP="$2"

if [ -z "$JETSON_IP" ] || [ -z "$MACBOOK_IP" ]; then
    echo "❌ Usage: $0 <jetson_ip> <macbook_ip>"
    echo "   Example: $0 192.168.1.100 192.168.1.200"
    exit 1
fi

echo "📝 Creating CycloneDDS configurations..."
echo "   Jetson IP: $JETSON_IP"
echo "   MacBook IP: $MACBOOK_IP"

# Create CycloneDDS config for MacBook
cat > docker/local/cyclonedds.xml << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Domain id="42">
    <Discovery>
      <Peers>
        <Peer address="$JETSON_IP"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
      <MaxMessageSize>65536</MaxMessageSize>
    </General>
  </Domain>
</CycloneDDS>
EOF

# Create CycloneDDS config for Jetson  
cat > docker/jetson/cyclonedx.xml << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Domain id="42">
    <Discovery>
      <Peers>
        <Peer address="$MACBOOK_IP"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
      <MaxMessageSize>65536</MaxMessageSize>
    </General>
  </Domain>
</CycloneDDS>
EOF

# Create setup script for CycloneDDS
cat > setup_cyclonedx.sh << EOF
#!/bin/bash
echo "🔄 Configuring ROS2 to use CycloneDDS..."

# Set environment variables  
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Set CycloneDDS config based on container
if [ "\$HOSTNAME" = "docker-desktop" ]; then
    export CYCLONEDX_URI=file:///app/docker/local/cyclonedx.xml
    echo "✅ MacBook container configured for CycloneDDS"
else
    export CYCLONEDX_URI=file:///app/docker/jetson/cyclonedx.xml
    echo "✅ Jetson container configured for CycloneDDS"
fi

echo ""
echo "🚀 CycloneDDS is now active!"
echo "   RMW_IMPLEMENTATION: \$RMW_IMPLEMENTATION"
echo "   CYCLONEDX_URI: \$CYCLONEDX_URI"
echo ""
echo "Test with: ros2 run demo_nodes_cpp talker/listener"
EOF

chmod +x setup_cyclonedx.sh

echo ""
echo "✅ CycloneDDS configurations created!"
echo ""
echo "📋 NEXT STEPS:"
echo ""
echo "1. UPDATE DOCKER COMPOSE FILES:"
echo "   Add to environment section:"
echo "     - RMW_IMPLEMENTATION=rmw_cyclonedx_cpp"
echo "     - CYCLONEDX_URI=/app/docker/local/cyclonedx.xml  (for MacBook)"
echo "     - CYCLONEDX_URI=/app/docker/jetson/cyclonedx.xml (for Jetson)"
echo ""
echo "2. RESTART CONTAINERS:"
echo "   sudo docker compose down && sudo docker compose up -d"
echo ""
echo "3. IN EACH CONTAINER:"
echo "   source /app/setup_cyclonedx.sh"
echo ""
echo "4. TEST COMMUNICATION:"
echo "   ros2 run demo_nodes_cpp talker/listener"
echo ""
echo "🎯 CycloneDDS is more reliable than FastRTPS for Docker multi-machine setups!"