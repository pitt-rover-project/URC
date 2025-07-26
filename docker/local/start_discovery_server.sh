#!/bin/bash
echo "🏃 Starting Discovery Server on MacBook..."
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="192.168.1.73:11888"

# Start discovery server
ros2 run fastdds discovery fastdiscoveryserver -i 0 -l 0.0.0.0 -p 11888 &
DISCOVERY_PID=$!

echo "✅ Discovery Server running on 192.168.1.73:11888 (PID: $DISCOVERY_PID)"
echo "   Kill with: kill $DISCOVERY_PID"
wait
