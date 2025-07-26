#!/bin/bash
echo "🎯 ULTIMATE ROS2 Multi-Machine Fix"
echo "=================================="

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}This script implements multiple fallback solutions${NC}"
echo ""

# Step 1: Disable macOS firewall
echo "🔥 Step 1: Disable macOS Firewall (TESTING ONLY)"
echo "------------------------------------------------"
echo "Current firewall state:"
/usr/libexec/ApplicationFirewall/socketfilterfw --getglobalstate 2>/dev/null

echo ""
echo "Disabling firewall for testing..."
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --setglobalstate off

# Step 2: Fix Docker containers with proper networking
echo ""
echo "🐳 Step 2: Restart Containers with Bridge Networking"
echo "---------------------------------------------------"
cd /Users/sandovalwilliamj/Developer/university-of-pittsburgh/rover-competition

echo "Stopping containers..."
sudo docker compose -f docker/local/docker-compose.yml down 2>/dev/null

echo "Starting with new network configuration..."
sudo docker compose -f docker/local/docker-compose.yml up -d

# Wait for containers to start
echo "Waiting for containers to initialize..."
sleep 10

# Step 3: Create a simple ROS2 test without Discovery Server
echo ""
echo "🚀 Step 3: Test Basic ROS2 Communication"
echo "----------------------------------------"

# Create a simple test script
cat > test_ros2_simple.sh << 'EOF'
#!/bin/bash
echo "🧪 Testing ROS2 Communication (Simple Method)"
echo "============================================="

# Set minimal environment
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Remove any custom profiles temporarily
unset FASTRTPS_DEFAULT_PROFILES_FILE

echo "Environment set:"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"  
echo "  RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo ""

echo "Starting ROS2 daemon..."
ros2 daemon stop 2>/dev/null
sleep 2
ros2 daemon start
sleep 3

echo "Testing topic list..."
ros2 topic list

echo ""
echo "🎯 NOW TEST:"
echo "MacBook: ros2 run demo_nodes_cpp talker"
echo "Jetson:  ros2 run demo_nodes_cpp listener"
EOF

chmod +x test_ros2_simple.sh

# Step 4: Create CycloneDDS alternative
echo ""
echo "🔄 Step 4: Prepare CycloneDDS Alternative"
echo "-----------------------------------------"

# Create CycloneDDS config that forces unicast
cat > docker/local/cyclonedds_unicast.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="42">
    <Discovery>
      <Peers>
        <Peer address="192.168.1.91"/>
        <Peer address="192.168.1.1"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
      <EnableMulticastLoopback>false</EnableMulticastLoopback>
    </General>
  </Domain>
</CycloneDX>
EOF

# Create CycloneDDS test script
cat > test_cyclonedx.sh << 'EOF'
#!/bin/bash
echo "🌀 Testing with CycloneDDS"
echo "=========================="

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export CYCLONEDX_URI=file:///app/docker/local/cyclonedx_unicast.xml

echo "CycloneDDS environment configured"
echo "Testing..."

ros2 daemon stop 2>/dev/null
sleep 2
ros2 daemon start
sleep 3

echo "🎯 NOW TEST with CycloneDDS:"
echo "MacBook: ros2 run demo_nodes_cpp talker"
echo "Jetson:  ros2 run demo_nodes_cpp listener"
EOF

chmod +x test_cyclonedx.sh

echo ""
echo "🎯 ULTIMATE FIX READY!"
echo "====================="
echo ""
echo -e "${GREEN}✅ COMPLETED:${NC}"
echo "   1. Disabled macOS firewall"
echo "   2. Restarted containers with bridge networking"
echo "   3. Created simple ROS2 test (./test_ros2_simple.sh)"
echo "   4. Created CycloneDDS fallback (./test_cyclonedx.sh)"
echo ""
echo -e "${YELLOW}📋 TRY THESE IN ORDER:${NC}"
echo ""
echo "🥇 FIRST: Run connectivity test"
echo "   ./debug_connectivity.sh"
echo ""
echo "🥈 SECOND: Try simple ROS2 (in both containers)"
echo "   ./test_ros2_simple.sh"
echo "   ros2 run demo_nodes_cpp talker/listener"
echo ""
echo "🥉 THIRD: Try CycloneDDS (if FastRTPS fails)"
echo "   ./test_cyclonedx.sh"
echo "   ros2 run demo_nodes_cpp talker/listener"
echo ""
echo -e "${RED}🚨 IMPORTANT:${NC}"
echo "   - Run test scripts in BOTH containers"
echo "   - Test with talker on Jetson, listener on MacBook"
echo "   - If still fails, we'll try host networking bypass"
echo ""