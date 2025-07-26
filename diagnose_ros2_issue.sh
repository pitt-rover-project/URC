#!/bin/bash
echo "🔍 ROS2 Multi-Machine Communication Diagnostics"
echo "==============================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Running diagnostics... This may take 30 seconds${NC}"
echo ""

# Test 1: Basic network connectivity
echo "📡 Test 1: Network Connectivity"
echo "--------------------------------"
if command -v hostname >/dev/null 2>&1; then
    MY_IP=$(hostname -I | awk '{print $1}')
    echo "✅ My IP address: $MY_IP"
else
    echo "❌ Cannot determine IP address"
fi

# Test 2: ROS2 Environment
echo ""
echo "🔧 Test 2: ROS2 Environment"
echo "---------------------------"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-'not set'}"
echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-'not set'}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-'not set'}"
echo "FASTRTPS_DEFAULT_PROFILES_FILE: ${FASTRTPS_DEFAULT_PROFILES_FILE:-'not set'}"

# Test 3: XML Profile validation
echo ""
echo "📄 Test 3: FastRTPS Profile Validation"
echo "--------------------------------------"
if [ -n "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    if [ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
        echo -e "${GREEN}✅ Profile file exists: $FASTRTPS_DEFAULT_PROFILES_FILE${NC}"
        if xmllint --noout "$FASTRTPS_DEFAULT_PROFILES_FILE" 2>/dev/null; then
            echo -e "${GREEN}✅ XML syntax is valid${NC}"
        else
            echo -e "${RED}❌ XML syntax errors detected${NC}"
        fi
    else
        echo -e "${RED}❌ Profile file not found: $FASTRTPS_DEFAULT_PROFILES_FILE${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  No FastRTPS profile configured${NC}"
fi

# Test 4: UDP port testing
echo ""
echo "🌐 Test 4: UDP Port Connectivity"
echo "--------------------------------"
echo "Testing DDS discovery port 7401..."
timeout 5 nc -u -l 7401 </dev/null >/dev/null 2>&1 &
NC_PID=$!
sleep 1
if ps -p $NC_PID > /dev/null; then
    echo -e "${GREEN}✅ Can bind to UDP port 7401${NC}"
    kill $NC_PID 2>/dev/null
else
    echo -e "${RED}❌ Cannot bind to UDP port 7401${NC}"
fi

# Test 5: ROS2 multicast test
echo ""
echo "📻 Test 5: Multicast Capability"
echo "-------------------------------"
if command -v ros2 >/dev/null 2>&1; then
    echo "Testing ROS2 multicast (5 second timeout)..."
    timeout 5 ros2 multicast receive &
    MULTICAST_PID=$!
    sleep 1
    echo "test multicast" | timeout 3 ros2 multicast send 2>/dev/null
    wait $MULTICAST_PID 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Multicast working${NC}"
    else
        echo -e "${RED}❌ Multicast not working${NC}"
    fi
else
    echo -e "${RED}❌ ros2 command not available${NC}"
fi

# Test 6: Local ROS2 discovery
echo ""
echo "🔍 Test 6: ROS2 Node Discovery"
echo "------------------------------"
if command -v ros2 >/dev/null 2>&1; then
    echo "Testing local node discovery..."
    ros2 daemon stop >/dev/null 2>&1
    sleep 2
    ros2 daemon start >/dev/null 2>&1
    sleep 3
    
    NODES=$(ros2 node list 2>/dev/null | wc -l)
    if [ "$NODES" -gt 0 ]; then
        echo -e "${GREEN}✅ Found $NODES ROS2 nodes${NC}"
        ros2 node list
    else
        echo -e "${RED}❌ No ROS2 nodes discovered${NC}"
    fi
else
    echo -e "${RED}❌ ros2 command not available${NC}"
fi

echo ""
echo "🏁 DIAGNOSIS COMPLETE"
echo "===================="
echo ""
echo -e "${YELLOW}📊 RECOMMENDED ACTIONS:${NC}"
echo ""
if [ -z "$MY_IP" ]; then
    echo -e "${RED}🔴 CRITICAL: Fix network configuration first${NC}"
fi

if [ -n "$FASTRTPS_DEFAULT_PROFILES_FILE" ] && [ ! -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    echo -e "${RED}🔴 CRITICAL: FastRTPS profile file missing${NC}"
fi

echo -e "${GREEN}🟢 Try these fixes in order:${NC}"
echo "   1. ./fix_macos_firewall.sh"
echo "   2. ./setup_discovery_server.sh <jetson_ip> <macbook_ip>"
echo "   3. Switch to CycloneDDS (see agent recommendations)"
echo ""
echo -e "${YELLOW}💡 Run this script on BOTH machines to compare results${NC}"