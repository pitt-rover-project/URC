# ROS2 Communication Diagnostic Plan
## MacBook Docker ↔ Jetson Docker Communication Failure

### Overview
This diagnostic plan provides a systematic approach to identify and resolve ROS2 communication failures between MacBook Docker containers and Jetson Docker containers in the URC project.

### Current Configuration Analysis
- **Both containers**: Using `network_mode: host` (bypassing Docker networking)
- **ROS_DOMAIN_ID**: 0 (same domain)
- **ROS_LOCALHOST_ONLY**: 0 (allows remote communication)
- **DDS Implementation**: FastRTPS/FastDDS on both systems
- **Transport**: Custom UDPv4 profile with 0.0.0.0 whitelist

---

## PHASE 1: BASIC NETWORK CONNECTIVITY

### Test 1.1: Physical Network Connectivity
**Purpose**: Verify basic IP connectivity between devices

**Commands on MacBook**:
```bash
# Get MacBook IP address
ip addr show | grep inet | grep -v 127.0.0.1 | grep -v ::1
# OR on macOS:
ifconfig | grep "inet " | grep -v 127.0.0.1
```

**Commands on Jetson**:
```bash
# Get Jetson IP address
ip addr show | grep inet | grep -v 127.0.0.1 | grep -v ::1
```

**Cross-ping Test**:
```bash
# From MacBook to Jetson
ping -c 5 <JETSON_IP>

# From Jetson to MacBook  
ping -c 5 <MACBOOK_IP>
```

**Expected Results**:
- ✅ PASS: Both devices respond to ping with <5ms latency
- ❌ FAIL: No response or high latency (>100ms)

**Next Steps**:
- PASS → Proceed to Test 1.2
- FAIL → Check network routing, firewall, WiFi/Ethernet configuration

### Test 1.2: UDP Port Connectivity
**Purpose**: Verify UDP port accessibility for DDS communication

**Commands on Jetson (listener)**:
```bash
# Listen on DDS discovery port
nc -ul 7400
```

**Commands on MacBook (sender)**:
```bash
# Send test message to DDS discovery port
echo "test_message" | nc -u <JETSON_IP> 7400
```

**Alternative using Python**:
```bash
# On receiving device (Jetson)
python3 -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', 7400))
print('Listening on port 7400...')
data, addr = s.recvfrom(1024)
print(f'Received: {data} from {addr}')
"

# On sending device (MacBook)
python3 -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(b'test_message', ('<JETSON_IP>', 7400))
print('Message sent')
"
```

**Expected Results**:
- ✅ PASS: Message received on listener
- ❌ FAIL: No message received

**Next Steps**:
- PASS → Proceed to Test 1.3
- FAIL → Check firewall rules, Docker host networking

### Test 1.3: Docker Host Network Validation
**Purpose**: Verify Docker containers can communicate using host networking

**Commands in MacBook Container**:
```bash
docker exec -it pitt_urc_local bash
# Inside container - verify host network access
ip addr show
netstat -tuln | grep 7400
```

**Commands in Jetson Container**:
```bash
docker exec -it pitt_urc_jetson bash
# Inside container - verify host network access  
ip addr show
netstat -tuln | grep 7400
```

**Expected Results**:
- ✅ PASS: Container IP matches host IP, can access host network interfaces
- ❌ FAIL: Container isolated from host network

**Next Steps**:
- PASS → Proceed to Phase 2
- FAIL → Fix Docker host networking configuration

---

## PHASE 2: ROS2 DISCOVERY DIAGNOSTICS

### Test 2.1: ROS2 Environment Validation
**Purpose**: Verify ROS2 environment is properly configured in both containers

**Commands in Both Containers**:
```bash
# Verify ROS2 installation
ros2 --version

# Check environment variables
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"  
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "FASTRTPS_DEFAULT_PROFILES_FILE: $FASTRTPS_DEFAULT_PROFILES_FILE"

# Verify DDS profile file exists
ls -la $FASTRTPS_DEFAULT_PROFILES_FILE

# Test basic ROS2 functionality
ros2 topic list
```

**Expected Results**:
- ✅ PASS: ROS2 Humble detected, all env vars correct, profile file exists
- ❌ FAIL: Missing ROS2, incorrect env vars, or missing profile

**Next Steps**:
- PASS → Proceed to Test 2.2
- FAIL → Fix ROS2 installation or environment setup

### Test 2.2: Local ROS2 Node Communication
**Purpose**: Verify ROS2 works locally within each container

**Commands in MacBook Container**:
```bash
# Terminal 1 - Start talker
ros2 run demo_nodes_cpp talker

# Terminal 2 - Start listener  
ros2 run demo_nodes_cpp listener
```

**Commands in Jetson Container**:
```bash
# Terminal 1 - Start talker
ros2 run demo_nodes_cpp talker

# Terminal 2 - Start listener
ros2 run demo_nodes_cpp listener
```

**Expected Results**:
- ✅ PASS: Listener receives messages from talker in same container
- ❌ FAIL: No communication within container

**Next Steps**:
- PASS → Proceed to Test 2.3
- FAIL → Debug local ROS2/DDS configuration

### Test 2.3: ROS2 Node Discovery Detection
**Purpose**: Determine if nodes can discover each other across containers

**Commands - Start nodes in both containers**:
```bash
# In MacBook container
ros2 run demo_nodes_cpp talker --ros-args -r __node:=mac_talker

# In Jetson container  
ros2 run demo_nodes_cpp talker --ros-args -r __node:=jetson_talker
```

**Commands - Check node discovery**:
```bash
# In both containers
ros2 node list
ros2 topic list
ros2 topic info /chatter
```

**Expected Results**:
- ✅ PASS: Both nodes visible in `ros2 node list` from both containers
- ❌ FAIL: Only local node visible from each container

**Next Steps**:
- PASS → Proceed to Test 2.4  
- FAIL → Proceed to Phase 3 (DDS Transport Debug)

### Test 2.4: Cross-Container Topic Subscription
**Purpose**: Test if remote topic subscription works

**Commands**:
```bash
# In MacBook container
ros2 run demo_nodes_cpp talker

# In Jetson container
ros2 run demo_nodes_cpp listener  
ros2 topic echo /chatter
```

**Expected Results**:
- ✅ PASS: Jetson listener receives messages from MacBook talker
- ❌ FAIL: No messages received despite node discovery working

**Next Steps**:
- PASS → Communication working! Skip to Phase 5 (Verification)
- FAIL → Proceed to Phase 3 (DDS Transport Debug)

---

## PHASE 3: DDS TRANSPORT LAYER DEBUGGING

### Test 3.1: DDS Discovery Traffic Analysis
**Purpose**: Monitor actual DDS discovery packets

**Commands on MacBook**:
```bash
# Monitor UDP traffic on discovery ports
sudo tcpdump -i any -n udp and port 7400
# Alternative ports to monitor: 7401, 7410, 7411
```

**Commands on Jetson**:
```bash
# Monitor UDP traffic on discovery ports  
sudo tcpdump -i any -n udp and port 7400
```

**Start ROS2 nodes while monitoring**:
```bash
# Start nodes in both containers and observe traffic
ros2 run demo_nodes_cpp talker
```

**Expected Results**:
- ✅ PASS: UDP discovery packets visible between IP addresses
- ❌ FAIL: No UDP traffic observed

**Next Steps**:
- PASS → Proceed to Test 3.2
- FAIL → Check firewall, network routing

### Test 3.2: FastDDS Discovery Debug Logging
**Purpose**: Enable detailed DDS logging to trace discovery process

**Commands in Both Containers**:
```bash
# Enable FastDDS debug logging
export FASTDDS_LOG_LEVEL=Log::Kind::Info

# Start talker with debug output
ros2 run demo_nodes_cpp talker 2>&1 | tee /tmp/fastdds_debug.log
```

**Analysis Commands**:
```bash
# Search for discovery-related messages
grep -i "discover\|participant\|endpoint" /tmp/fastdds_debug.log
```

**Expected Results**:
- ✅ PASS: Discovery messages showing remote participant detection
- ❌ FAIL: No remote participant discovery messages

**Next Steps**:
- PASS → Discovery working, check data transport (Test 3.3)
- FAIL → Fix DDS transport configuration

### Test 3.3: Custom DDS Profile Testing
**Purpose**: Test with simplified DDS configuration

**Commands - Create test profile**:
```bash
# Create simplified DDS profile
cat > /tmp/test_profile.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="default_participant">
        <rtps>
            <useBuiltinTransports>true</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF

# Test with simplified profile
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/test_profile.xml
ros2 run demo_nodes_cpp talker
```

**Expected Results**:
- ✅ PASS: Communication works with builtin transports
- ❌ FAIL: Still no communication

**Next Steps**:
- PASS → Original profile has issues, use simplified version
- FAIL → Proceed to Test 3.4

### Test 3.4: Alternative DDS Implementation Test
**Purpose**: Test with CycloneDDS instead of FastDDS

**Commands in Both Containers**:
```bash
# Switch to CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE

# Test communication
ros2 run demo_nodes_cpp talker   # MacBook
ros2 run demo_nodes_cpp listener # Jetson
```

**Expected Results**:
- ✅ PASS: Communication works with CycloneDDS
- ❌ FAIL: Still no communication

**Next Steps**:
- PASS → FastDDS configuration issue identified
- FAIL → Proceed to Phase 4

---

## PHASE 4: CONTAINER NETWORKING DEEP DIVE

### Test 4.1: Container Network Interface Analysis
**Purpose**: Verify host network mode is working correctly

**Commands in Both Containers**:
```bash
# Compare container vs host network interfaces
docker exec pitt_urc_local ip addr show > /tmp/container_net.txt
ip addr show > /tmp/host_net.txt
diff /tmp/container_net.txt /tmp/host_net.txt
```

**Expected Results**:
- ✅ PASS: No differences (host networking working)
- ❌ FAIL: Different network interfaces

### Test 4.2: Firewall and iptables Analysis
**Purpose**: Check for firewall rules blocking DDS

**Commands on Both Hosts**:
```bash
# Check iptables rules
sudo iptables -L -n -v

# Check for Docker-related rules
sudo iptables -t nat -L -n -v

# On macOS, check pfctl instead
sudo pfctl -sr
```

**Expected Results**:
- ✅ PASS: No blocking rules found
- ❌ FAIL: Rules blocking UDP 7400-7411 ports

### Test 4.3: Multi-Interface Network Test
**Purpose**: Test specific network interface binding

**Commands**:
```bash
# Force binding to specific interface
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/interface_profile.xml

# Create interface-specific profile
cat > /tmp/interface_profile.xml << EOF
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList>
                <address><SPECIFIC_INTERFACE_IP></address>
            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
</profiles>
EOF
```

---

## PHASE 5: COMPREHENSIVE LOGGING AND DEBUGGING

### Test 5.1: Complete ROS2 Communication Trace
**Purpose**: Capture complete communication trace for analysis

**Setup Commands**:
```bash
# Enable maximum verbosity
export ROS_LOG_LEVEL=DEBUG  
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0

# Enable FastDDS detailed logging
export FASTDDS_LOG_LEVEL=Log::Kind::Debug

# Start comprehensive logging
mkdir -p /tmp/ros2_debug
```

**Execution Commands**:
```bash
# MacBook Container
ros2 run demo_nodes_cpp talker 2>&1 | tee /tmp/ros2_debug/mac_talker.log &

# Jetson Container  
ros2 run demo_nodes_cpp listener 2>&1 | tee /tmp/ros2_debug/jetson_listener.log &

# Network trace
sudo tcpdump -i any -w /tmp/ros2_debug/network_trace.pcap udp &

# Let run for 30 seconds then analyze
sleep 30
pkill tcpdump
```

### Test 5.2: Log Analysis Scripts
**Purpose**: Automated analysis of debug logs

**Analysis Commands**:
```bash
# Check for discovery messages
grep -i "participant.*discovered\|endpoint.*discovered" /tmp/ros2_debug/*.log

# Check for transport errors
grep -i "error\|fail\|timeout" /tmp/ros2_debug/*.log

# Network packet analysis (requires wireshark/tshark)
tshark -r /tmp/ros2_debug/network_trace.pcap -Y "udp.port==7400" -V
```

---

## SYSTEMATIC TROUBLESHOOTING FLOWCHART

```
START: ROS2 Communication Failure
│
├── Phase 1: Network Connectivity
│   ├── Test 1.1: Ping Test
│   │   ├── PASS → Test 1.2
│   │   └── FAIL → Fix network/routing
│   ├── Test 1.2: UDP Port Test
│   │   ├── PASS → Test 1.3  
│   │   └── FAIL → Check firewall
│   └── Test 1.3: Docker Network Test
│       ├── PASS → Phase 2
│       └── FAIL → Fix Docker networking
│
├── Phase 2: ROS2 Discovery
│   ├── Test 2.1: Environment Check
│   │   ├── PASS → Test 2.2
│   │   └── FAIL → Fix ROS2 setup
│   ├── Test 2.2: Local Communication
│   │   ├── PASS → Test 2.3
│   │   └── FAIL → Debug local ROS2
│   ├── Test 2.3: Node Discovery
│   │   ├── PASS → Test 2.4
│   │   └── FAIL → Phase 3
│   └── Test 2.4: Topic Subscription  
│       ├── PASS → SOLVED!
│       └── FAIL → Phase 3
│
├── Phase 3: DDS Transport Debug
│   ├── Test 3.1: Traffic Analysis
│   ├── Test 3.2: Debug Logging
│   ├── Test 3.3: Custom Profile
│   └── Test 3.4: Alternative DDS
│
├── Phase 4: Container Networking
│   ├── Test 4.1: Interface Analysis
│   ├── Test 4.2: Firewall Check
│   └── Test 4.3: Multi-Interface Test
│
└── Phase 5: Comprehensive Debug
    ├── Test 5.1: Complete Trace
    └── Test 5.2: Log Analysis
```

---

## VERIFICATION STEPS FOR EACH FIX ATTEMPT

### After Network Fixes:
1. Re-run ping test (Test 1.1)
2. Re-run UDP connectivity (Test 1.2)  
3. Verify with basic ROS2 communication test

### After ROS2/DDS Configuration Fixes:
1. Test local communication first (Test 2.2)
2. Test node discovery (Test 2.3)
3. Test cross-container topic subscription (Test 2.4)
4. Verify with multiple topics and node types

### After Container Networking Fixes:
1. Verify container network interface matches host
2. Test UDP connectivity again
3. Full ROS2 communication test

### Final Verification Protocol:
1. **Multi-node Test**: Run talker/listener pair in both directions
2. **Service Test**: Test ROS2 service calls across containers
3. **Action Test**: Test ROS2 action servers/clients across containers  
4. **High-frequency Test**: Test high-rate topic publishing (100Hz)
5. **Stress Test**: Multiple simultaneous topics and services

---

## COMMON SOLUTIONS BASED ON DIAGNOSTIC RESULTS

### If Network Connectivity Fails:
- Check WiFi/Ethernet configuration
- Verify devices on same subnet
- Check router/switch configuration
- Test with direct Ethernet connection

### If UDP Ports Blocked:
```bash
# Open required ports (Linux)
sudo ufw allow 7400:7411/udp

# macOS firewall
sudo pfctl -f /etc/pf.conf
```

### If Docker Host Networking Fails:
```bash
# Restart Docker with proper networking
docker-compose down
docker-compose up -d
```

### If FastDDS Profile Issues:
- Replace with simplified builtin transport profile
- Or switch to CycloneDDS entirely

### If Discovery Works But Data Transfer Fails:
- Check for MTU size issues  
- Test with smaller message payloads
- Verify data transport port ranges (7410-7411)

This comprehensive diagnostic plan should systematically identify the exact cause of your ROS2 communication failure between MacBook Docker and Jetson Docker containers.