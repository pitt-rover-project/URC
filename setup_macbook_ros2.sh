#!/bin/bash
echo "🧹 STEP 1: Cleaning MacBook Container ROS2 Environment"
echo "======================================================"

# Kill any existing ROS2 processes
echo "Stopping existing ROS2 processes..."
pkill -f demo_nodes_cpp 2>/dev/null || true
pkill -f talker 2>/dev/null || true
pkill -f listener 2>/dev/null || true
pkill -f fastdiscoveryserver 2>/dev/null || true

# Stop ROS2 daemon
echo "Stopping ROS2 daemon..."
ros2 daemon stop 2>/dev/null || true
sleep 3

echo ""
echo "🔧 STEP 2: Configure Minimal ROS2 Environment"
echo "=============================================="

# Set minimal ROS2 environment (SAME AS JETSON)
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Remove any custom FastDDS profiles
unset FASTRTPS_DEFAULT_PROFILES_FILE

# Source ROS2
source /opt/ros/humble/setup.bash

echo "✅ Environment configured:"
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "   ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "   FASTRTPS_DEFAULT_PROFILES_FILE: ${FASTRTPS_DEFAULT_PROFILES_FILE:-'(unset)'}"

echo ""
echo "🔄 STEP 3: Start ROS2 Daemon"
echo "============================"

# Start ROS2 daemon with clean environment
ros2 daemon start
sleep 5

echo ""
echo "🔍 STEP 4: Test Topic Discovery"
echo "==============================="

echo "Current topics visible:"
ros2 topic list

echo ""
echo "📊 Looking for Jetson topics..."
if ros2 topic list | grep -q "/chatter"; then
    echo "✅ Found /chatter topic from Jetson!"
    ros2 topic info /chatter
else
    echo "❌ /chatter topic not found - Jetson may not be publishing"
    echo ""
    echo "💡 TROUBLESHOOTING:"
    echo "   1. Verify Jetson talker is running: ros2 run demo_nodes_cpp talker"
    echo "   2. Check Jetson uses same domain: export ROS_DOMAIN_ID=42"
    echo "   3. Both should be on same network: 192.168.1.x"
fi

echo ""
echo "🎯 NEXT STEPS:"
echo "=============="
echo "To start listener: ros2 run demo_nodes_cpp listener"
echo "To start talker:   ros2 run demo_nodes_cpp talker"
echo ""
echo "🚨 IMPORTANT: Run these commands in this SAME terminal"
echo "       to preserve the environment variables!"
