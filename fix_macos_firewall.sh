#!/bin/bash
echo "🔥 Fixing macOS Firewall for ROS2 DDS Communication"
echo "=================================================="

# Check current firewall status
echo "Current firewall status:"
/usr/libexec/ApplicationFirewall/socketfilterfw --getglobalstate --getblockall --getallowsigned --getstealthmode

echo ""
echo "⚠️  TEMPORARILY DISABLING FIREWALL FOR TESTING..."
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --setglobalstate off

echo "✅ Firewall disabled. Test ROS2 communication now!"
echo ""
echo "🚨 REMEMBER TO RE-ENABLE FIREWALL AFTER TESTING:"
echo "   sudo /usr/libexec/ApplicationFirewall/socketfilterfw --setglobalstate on"
echo ""

# Test UDP multicast capability
echo "Testing UDP multicast capability..."
nc -u -l 7401 &
NC_PID=$!
sleep 1
echo "test" | nc -u 239.255.0.1 7401
kill $NC_PID 2>/dev/null

echo ""
echo "📋 Next steps:"
echo "1. Test ROS2 communication: ros2 run demo_nodes_cpp talker/listener"
echo "2. If it works, firewall was the issue!"
echo "3. Re-enable firewall and configure proper rules"