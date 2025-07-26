#!/bin/bash
# ROS2 Communication Quick Diagnostic Script
# University of Pittsburgh Rover Challenge Project

set -e

echo "================================="
echo "ROS2 Communication Quick Diagnostic"
echo "================================="
echo

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print test results
print_result() {
    local test_name=$1
    local result=$2
    local details=$3
    
    if [ "$result" = "PASS" ]; then
        echo -e "${GREEN}✅ PASS${NC}: $test_name"
    elif [ "$result" = "FAIL" ]; then
        echo -e "${RED}❌ FAIL${NC}: $test_name"
    else
        echo -e "${YELLOW}⚠️  WARN${NC}: $test_name"
    fi
    
    if [ -n "$details" ]; then
        echo -e "   ${BLUE}→${NC} $details"
    fi
    echo
}

# Check if running in container
check_container() {
    echo "=== ENVIRONMENT CHECK ==="
    
    if [ -f /.dockerenv ]; then
        print_result "Container Detection" "PASS" "Running inside Docker container"
        CONTAINER_NAME=$(hostname)
        echo -e "${BLUE}Container Name:${NC} $CONTAINER_NAME"
    else
        print_result "Container Detection" "WARN" "Running on host system"
    fi
    
    # Check IP addresses
    echo -e "${BLUE}Network Interfaces:${NC}"
    if command -v ip >/dev/null 2>&1; then
        ip addr show | grep -E "inet [0-9]" | grep -v "127.0.0.1" | head -3
    else
        ifconfig | grep -E "inet [0-9]" | grep -v "127.0.0.1" | head -3
    fi
    echo
}

# Check ROS2 environment
check_ros2_env() {
    echo "=== ROS2 ENVIRONMENT CHECK ==="
    
    if command -v ros2 >/dev/null 2>&1; then
        print_result "ROS2 Installation" "PASS" "$(ros2 --version)"
        
        # Check environment variables
        echo -e "${BLUE}ROS2 Environment:${NC}"
        echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-'NOT SET'}"
        echo "  ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-'NOT SET'}"
        echo "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-'NOT SET'}"
        echo "  FASTRTPS_DEFAULT_PROFILES_FILE: ${FASTRTPS_DEFAULT_PROFILES_FILE:-'NOT SET'}"
        
        # Check DDS profile file
        if [ -n "$FASTRTPS_DEFAULT_PROFILES_FILE" ] && [ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
            print_result "FastDDS Profile File" "PASS" "$FASTRTPS_DEFAULT_PROFILES_FILE exists"
        else
            print_result "FastDDS Profile File" "WARN" "Profile file not found or not set"
        fi
        
    else
        print_result "ROS2 Installation" "FAIL" "ros2 command not found"
        echo "Please ensure ROS2 is installed and sourced"
        exit 1
    fi
    echo
}

# Test local ROS2 functionality
test_local_ros2() {
    echo "=== LOCAL ROS2 FUNCTIONALITY TEST ==="
    
    # Test basic ROS2 commands
    if ros2 topic list >/dev/null 2>&1; then
        print_result "ROS2 Topic List" "PASS" "Basic ROS2 functionality working"
    else
        print_result "ROS2 Topic List" "FAIL" "ROS2 topic list command failed"
        return 1
    fi
    
    # Test node discovery
    echo -e "${BLUE}Current ROS2 Nodes:${NC}"
    ros2 node list 2>/dev/null || echo "  No nodes currently running"
    
    echo -e "${BLUE}Current ROS2 Topics:${NC}"
    ros2 topic list 2>/dev/null | head -10
    echo
}

# Network connectivity test
test_network_connectivity() {
    echo "=== NETWORK CONNECTIVITY TEST ==="
    
    echo "Please enter the IP address of the remote device to test connectivity:"
    read -p "Remote IP (or press Enter to skip): " REMOTE_IP
    
    if [ -z "$REMOTE_IP" ]; then
        print_result "Network Test" "WARN" "Skipped - no remote IP provided"
        return 0
    fi
    
    # Ping test
    if ping -c 3 "$REMOTE_IP" >/dev/null 2>&1; then
        print_result "Ping Test" "PASS" "$REMOTE_IP is reachable"
    else
        print_result "Ping Test" "FAIL" "$REMOTE_IP is not reachable"
        return 1
    fi
    
    # UDP port test (basic check if nc is available)
    if command -v nc >/dev/null 2>&1; then
        echo -e "${BLUE}UDP Port Test:${NC} Use 'nc -ul 7400' on remote device, then press Enter"
        read -p "Press Enter when ready..."
        
        # Send test message
        if echo "diagnostic_test" | timeout 5 nc -u "$REMOTE_IP" 7400 2>/dev/null; then
            print_result "UDP Port 7400 Test" "PASS" "Port appears to be open"
        else
            print_result "UDP Port 7400 Test" "WARN" "Could not confirm port accessibility"
        fi
    else
        print_result "UDP Port Test" "WARN" "netcat (nc) not available for port testing"
    fi
    echo
}

# ROS2 node discovery test
test_ros2_discovery() {
    echo "=== ROS2 NODE DISCOVERY TEST ==="
    
    echo "Starting temporary ROS2 talker for discovery test..."
    echo "This will run for 10 seconds to allow remote nodes to discover it"
    
    # Start talker in background
    timeout 10 ros2 run demo_nodes_cpp talker --ros-args -r __node:=diagnostic_talker >/dev/null 2>&1 &
    TALKER_PID=$!
    
    sleep 2
    
    # Check if talker is visible
    if ros2 node list | grep -q diagnostic_talker; then
        print_result "Local Node Creation" "PASS" "diagnostic_talker node created successfully"
    else
        print_result "Local Node Creation" "FAIL" "Failed to create diagnostic_talker node"
    fi
    
    # Wait for discovery period
    echo "Waiting for discovery period..."
    sleep 8
    
    # Check for other nodes (would indicate remote discovery)
    NODE_COUNT=$(ros2 node list | wc -l)
    if [ "$NODE_COUNT" -gt 1 ]; then
        print_result "Remote Node Discovery" "PASS" "Found $NODE_COUNT total nodes"
        echo -e "${BLUE}Discovered Nodes:${NC}"
        ros2 node list | sed 's/^/  /'
    else
        print_result "Remote Node Discovery" "FAIL" "Only local node visible"
    fi
    
    # Clean up
    kill $TALKER_PID 2>/dev/null || true
    echo
}

# DDS transport diagnostic
test_dds_transport() {
    echo "=== DDS TRANSPORT DIAGNOSTIC ==="
    
    # Check for DDS-related processes
    if pgrep -f "rmw_fastrtps\|rmw_cyclone" >/dev/null; then
        print_result "DDS Processes" "PASS" "DDS-related processes found"
    else
        print_result "DDS Processes" "WARN" "No obvious DDS processes running"
    fi
    
    # Test with debug logging
    echo "Testing with DDS debug logging enabled..."
    export FASTDDS_LOG_LEVEL=Log::Kind::Info
    
    # Brief test with debug output
    echo -e "${BLUE}Starting brief DDS debug test (5 seconds):${NC}"
    timeout 5 ros2 run demo_nodes_cpp talker 2>&1 | grep -E -i "(discover|participant|endpoint|transport)" | head -5 || true
    
    unset FASTDDS_LOG_LEVEL
    echo
}

# Generate diagnostic report
generate_report() {
    echo "=== DIAGNOSTIC SUMMARY ==="
    
    REPORT_FILE="/tmp/ros2_diagnostic_report_$(date +%Y%m%d_%H%M%S).txt"
    
    cat > "$REPORT_FILE" << EOF
ROS2 Communication Diagnostic Report
Generated: $(date)
Hostname: $(hostname)
Container: ${CONTAINER_NAME:-'N/A'}

=== Environment ===
ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-'NOT SET'}
ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-'NOT SET'}
RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-'NOT SET'}
FASTRTPS_DEFAULT_PROFILES_FILE: ${FASTRTPS_DEFAULT_PROFILES_FILE:-'NOT SET'}

=== Network Interfaces ===
$(ip addr show 2>/dev/null || ifconfig 2>/dev/null || echo "Network info not available")

=== ROS2 Status ===
$(ros2 --version 2>/dev/null || echo "ROS2 not available")
$(ros2 node list 2>/dev/null || echo "No nodes running")
$(ros2 topic list 2>/dev/null | head -10 || echo "No topics available")

EOF
    
    echo "Diagnostic report saved to: $REPORT_FILE"
    print_result "Report Generation" "PASS" "Full diagnostic data saved"
    echo
}

# Main execution
main() {
    echo "Starting ROS2 communication diagnostic..."
    echo "This script will test basic ROS2 functionality and network connectivity"
    echo
    
    check_container
    check_ros2_env
    test_local_ros2
    test_network_connectivity
    test_ros2_discovery
    test_dds_transport
    generate_report
    
    echo "=== NEXT STEPS ==="
    echo "1. Run this script on both MacBook and Jetson containers"
    echo "2. Compare the results between both systems"
    echo "3. If issues found, refer to the full diagnostic plan:"
    echo "   /Users/sandovalwilliamj/Developer/university-of-pittsburgh/rover-competition/ROS2_Communication_Diagnostic_Plan.md"
    echo "4. For detailed troubleshooting, follow the systematic flowchart in the plan"
    echo
}

# Run main function
main "$@"