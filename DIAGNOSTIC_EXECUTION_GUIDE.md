# ROS2 Communication Diagnostic Execution Guide

## Quick Start - Run These Commands Immediately

### Step 1: Basic Environment Check
Run this on **both** MacBook and Jetson Docker containers:

```bash
# Enter your containers first
docker exec -it pitt_urc_local bash    # MacBook
docker exec -it pitt_urc_jetson bash   # Jetson

# Run the quick diagnostic
/app/ros2_quick_diagnostic.sh
```

### Step 2: UDP Connectivity Test
Run this **simultaneously** on both containers:

**On Jetson container (listener)**:
```bash
python3 /app/udp_connectivity_test.py --mode listen --duration 60
```

**On MacBook container (sender)**:
```bash
# Replace <JETSON_IP> with actual Jetson IP address
python3 /app/udp_connectivity_test.py --mode send --target-ip <JETSON_IP>
```

### Step 3: Simple ROS2 Communication Test
**Terminal 1 - MacBook container**:
```bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2 - Jetson container**:
```bash
ros2 run demo_nodes_cpp listener
```

**Terminal 3 - Check discovery (run on either container)**:
```bash
ros2 node list
ros2 topic list
ros2 topic info /chatter
```

---

## Systematic Diagnostic Execution

### Phase 1: Network Layer (5 minutes)

1. **Get IP Addresses**:
   ```bash
   # MacBook container
   ip addr show | grep "inet " | grep -v 127.0.0.1
   
   # Jetson container  
   ip addr show | grep "inet " | grep -v 127.0.0.1
   ```

2. **Basic Ping Test**:
   ```bash
   # From each container to the other
   ping -c 5 <OTHER_DEVICE_IP>
   ```

3. **UDP Port Test**:
   ```bash
   # Jetson: Listen on DDS discovery port
   nc -ul 7400
   
   # MacBook: Send test message
   echo "test_message" | nc -u <JETSON_IP> 7400
   ```

### Phase 2: ROS2 Environment (3 minutes)

1. **Environment Variables Check**:
   ```bash
   echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
   echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"  
   echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
   ```

2. **Local ROS2 Test**:
   ```bash
   # Test in same container
   ros2 run demo_nodes_cpp talker &
   ros2 run demo_nodes_cpp listener
   kill %1  # Stop background talker
   ```

### Phase 3: Discovery Debugging (10 minutes)

1. **Enable DDS Debug Logging**:
   ```bash
   export FASTDDS_LOG_LEVEL=Log::Kind::Info
   ros2 run demo_nodes_cpp talker 2>&1 | grep -i "discover\|participant"
   ```

2. **Network Traffic Analysis**:
   ```bash
   # Monitor DDS traffic (if tcpdump available)
   sudo tcpdump -i any -n udp and port 7400 &
   
   # Start ROS2 nodes and observe traffic
   ros2 run demo_nodes_cpp talker
   ```

3. **Node Discovery Test**:
   ```bash
   # Start nodes with unique names
   ros2 run demo_nodes_cpp talker --ros-args -r __node:=mac_talker    # MacBook
   ros2 run demo_nodes_cpp talker --ros-args -r __node:=jetson_talker # Jetson
   
   # Check if both are visible
   ros2 node list
   ```

---

## Expected Results and Actions

### ✅ SUCCESS INDICATORS:
- **Network**: Ping successful, UDP messages received
- **ROS2**: Both nodes visible in `ros2 node list` from both containers
- **Communication**: Listener receives messages from remote talker

### ❌ FAILURE PATTERNS:

#### Pattern 1: Network Connectivity Failed
**Symptoms**: Ping fails, no UDP traffic
**Actions**:
- Check WiFi/network configuration
- Verify both devices on same subnet
- Test with direct Ethernet connection

#### Pattern 2: ROS2 Discovery Failed  
**Symptoms**: Ping works, but only local nodes visible
**Actions**:
- Check firewall blocking UDP 7400-7411
- Test with simplified DDS profile
- Try CycloneDDS instead of FastDDS

#### Pattern 3: Discovery Works, Data Transfer Failed
**Symptoms**: Nodes visible but no message transfer
**Actions**:
- Check data transport ports (7410-7411)
- Test with smaller message payloads
- Verify MTU size settings

---

## Emergency Fixes

### Fix 1: Switch to CycloneDDS
```bash
# In both containers
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE

# Test communication
ros2 run demo_nodes_cpp talker   # MacBook
ros2 run demo_nodes_cpp listener # Jetson
```

### Fix 2: Simplified DDS Profile
```bash
# Create minimal profile
cat > /tmp/simple_profile.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="default_participant">
        <rtps>
            <useBuiltinTransports>true</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF

export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/simple_profile.xml
```

### Fix 3: Disable Localhost Only
```bash
# Ensure remote communication is enabled
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=0
```

---

## Verification Commands

After any fix attempt, run these to verify:

```bash
# 1. Cross-container talker/listener test
ros2 run demo_nodes_cpp talker     # MacBook
ros2 run demo_nodes_cpp listener   # Jetson

# 2. Service call test
ros2 run demo_nodes_cpp add_two_ints_server  # Jetson
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"  # MacBook

# 3. Topic echo test
ros2 topic pub /test_topic std_msgs/msg/String "data: 'Hello from MacBook'" --once  # MacBook
ros2 topic echo /test_topic  # Jetson
```

---

## Getting Help

If diagnostics show unexpected results:

1. **Check the comprehensive plan**: `/app/ROS2_Communication_Diagnostic_Plan.md`
2. **Save diagnostic logs**: All scripts generate timestamped log files in `/tmp/`
3. **Compare results**: Run diagnostics on both systems and compare outputs
4. **Escalate with data**: Include specific error messages and diagnostic outputs

## File Locations

All diagnostic files are in your project root:
- `/Users/sandovalwilliamj/Developer/university-of-pittsburgh/rover-competition/ROS2_Communication_Diagnostic_Plan.md`
- `/Users/sandovalwilliamj/Developer/university-of-pittsburgh/rover-competition/ros2_quick_diagnostic.sh`
- `/Users/sandovalwilliamj/Developer/university-of-pittsburgh/rover-competition/udp_connectivity_test.py`
- `/Users/sandovalwilliamj/Developer/university-of-pittsburgh/rover-competition/DIAGNOSTIC_EXECUTION_GUIDE.md`