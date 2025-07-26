#!/bin/bash
echo "🔍 DEEP DIVE: ROS2 Communication Diagnostics"
echo "============================================"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

JETSON_IP="192.168.1.91"
MAC_IP="192.168.1.1"

echo -e "${YELLOW}Testing network connectivity between:${NC}"
echo "   Jetson: $JETSON_IP"
echo "   Mac:    $MAC_IP"
echo ""

# Test 1: Basic ping
echo "🏓 Test 1: Basic Ping Connectivity"
echo "-----------------------------------"
ping -c 3 $JETSON_IP 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Can ping Jetson${NC}"
else
    echo -e "${RED}❌ Cannot ping Jetson${NC}"
fi

# Test 2: UDP port testing
echo ""
echo "🌐 Test 2: UDP Port Connectivity"
echo "--------------------------------"
echo "Testing Discovery Server port 11888..."

# Start a UDP listener on port 11888
timeout 10 nc -u -l 11888 &
NC_PID=$!
sleep 2

# Try to send data to it
echo "test_discovery" | timeout 3 nc -u $MAC_IP 11888 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ UDP port 11888 accessible${NC}"
else
    echo -e "${RED}❌ UDP port 11888 blocked${NC}"
fi

# Clean up
kill $NC_PID 2>/dev/null

# Test 3: ROS2 discovery ports
echo ""
echo "📻 Test 3: ROS2 DDS Ports (7400-7411)"
echo "-------------------------------------"
for port in 7400 7401 7411; do
    timeout 5 nc -u -l $port &
    NC_PID=$!
    sleep 1
    echo "test" | timeout 2 nc -u $MAC_IP $port 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Port $port accessible${NC}"
    else
        echo -e "${RED}❌ Port $port blocked${NC}"
    fi
    kill $NC_PID 2>/dev/null
done

# Test 4: Check if Discovery Server is actually running
echo ""
echo "🏗️  Test 4: Discovery Server Status"
echo "-----------------------------------"
if pgrep -f "fastdiscoveryserver" > /dev/null; then
    echo -e "${GREEN}✅ Discovery Server process found${NC}"
    echo "Discovery Server PIDs:"
    pgrep -f "fastdiscoveryserver" | xargs ps -p
else
    echo -e "${RED}❌ Discovery Server not running${NC}"
fi

# Test 5: Check container network configuration
echo ""
echo "🐳 Test 5: Container Network Configuration"
echo "-----------------------------------------"
echo "Container IP addresses:"
docker exec pitt_urc_local hostname -I 2>/dev/null || echo "MacBook container not running"
docker exec pitt_urc_jetson hostname -I 2>/dev/null || echo "Jetson container not accessible"

# Test 6: ROS2 environment check
echo ""
echo "🔧 Test 6: ROS2 Environment (MacBook Container)"
echo "----------------------------------------------"
docker exec pitt_urc_local bash -c '
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "ROS_DISCOVERY_SERVER: $ROS_DISCOVERY_SERVER"
' 2>/dev/null || echo "Cannot check MacBook container environment"

# Test 7: Test ROS2 node discovery within container
echo ""
echo "🔍 Test 7: ROS2 Node Discovery Test"
echo "-----------------------------------"
echo "Starting test nodes for 10 seconds..."

# Start talker in background
docker exec -d pitt_urc_local bash -c 'source /app/setup_ros2_client.sh && ros2 run demo_nodes_cpp talker' 2>/dev/null

sleep 5

# Check if listener can see the talker
docker exec pitt_urc_local bash -c 'source /app/setup_ros2_client.sh && timeout 5 ros2 topic list' 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ ROS2 topics discoverable in MacBook container${NC}"
else
    echo -e "${RED}❌ ROS2 discovery failing in MacBook container${NC}"
fi

# Clean up
docker exec pitt_urc_local pkill -f demo_nodes_cpp 2>/dev/null

echo ""
echo "🏁 DIAGNOSTIC SUMMARY"
echo "===================="
echo ""
echo -e "${YELLOW}📊 NEXT DEBUGGING STEPS:${NC}"
echo ""
echo "1. If ping fails: Network routing issue"
echo "2. If UDP ports blocked: Firewall issue"  
echo "3. If Discovery Server not running: Start it manually"
echo "4. If ROS2 discovery fails: Environment variable issue"
echo ""
echo -e "${GREEN}🚀 RECOMMENDED ACTIONS:${NC}"
echo "   1. Run: sudo /usr/libexec/ApplicationFirewall/socketfilterfw --setglobalstate off"
echo "   2. Manually start Discovery Server: cd docker/local && ./start_discovery_server.sh"
echo "   3. Test with simple UDP tools first, then ROS2"
echo ""