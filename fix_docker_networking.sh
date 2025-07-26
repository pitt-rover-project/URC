#!/bin/bash
echo "🔧 Fixing Docker Desktop Networking for ROS2"
echo "============================================="

echo "🔍 Current network status:"
echo "Docker container IPs:"
docker exec pitt_urc_local hostname -I 2>/dev/null || echo "Container not running"

echo ""
echo "Host Mac WiFi IP:"
ifconfig | grep "inet " | grep -v 127.0.0.1 | head -1 | awk '{print $2}'

echo ""
echo "🚨 PROBLEM IDENTIFIED:"
echo "   - Docker container: 192.168.65.x (Docker Desktop subnet)"
echo "   - WiFi network: 192.168.1.x (real network)"
echo "   - ROS2 multicast cannot cross subnets!"
echo ""

echo "💡 SOLUTIONS:"
echo ""
echo "OPTION A: Use Discovery Server (RECOMMENDED)"
echo "   - Bypasses multicast entirely"
echo "   - Works across any network topology"
echo "   - Run: ./setup_discovery_server.sh <jetson_ip> <host_mac_ip>"
echo ""
echo "OPTION B: Fix Docker networking"
echo "   - Stop Docker Desktop"
echo "   - Enable 'Use kernel networking for UDP' in Docker settings"
echo "   - OR switch to Docker with bridge networking"
echo ""

# Get the actual Mac IP on WiFi
MAC_WIFI_IP=$(ifconfig | grep "inet " | grep -v 127.0.0.1 | grep -v 192.168.65 | head -1 | awk '{print $2}')

if [ -n "$MAC_WIFI_IP" ]; then
    echo "✅ Found Mac WiFi IP: $MAC_WIFI_IP"
    echo ""
    echo "🚀 QUICK FIX COMMAND:"
    echo "   ./setup_discovery_server.sh 192.168.1.91 $MAC_WIFI_IP"
    echo ""
else
    echo "❌ Could not determine Mac WiFi IP"
fi

echo "📋 NEXT STEPS:"
echo "1. Confirm Jetson IP: docker exec pitt_urc_jetson hostname -I"
echo "2. Use Discovery Server with WIFI IPs (not Docker container IPs)"
echo "3. Test: ros2 run demo_nodes_cpp talker/listener"