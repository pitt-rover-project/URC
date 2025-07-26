#!/usr/bin/env python3
"""
UDP Connectivity Test for ROS2 DDS Communication
University of Pittsburgh Rover Challenge Project

This script tests UDP connectivity between MacBook and Jetson containers
on the ports used by ROS2 DDS discovery and data transport.
"""

import socket
import threading
import time
import argparse
import sys
from datetime import datetime

# DDS Discovery and data ports
DDS_PORTS = [7400, 7401, 7410, 7411, 7412]

class UDPConnectivityTester:
    def __init__(self, remote_ip=None):
        self.remote_ip = remote_ip
        self.test_results = {}
        
    def print_status(self, message, status="INFO"):
        """Print colored status messages"""
        colors = {
            "INFO": "\033[0;34m",   # Blue
            "PASS": "\033[0;32m",   # Green  
            "FAIL": "\033[0;31m",   # Red
            "WARN": "\033[1;33m"    # Yellow
        }
        reset = "\033[0m"
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"[{timestamp}] {colors.get(status, '')}{status}{reset}: {message}")
    
    def listen_on_port(self, port, duration=10):
        """Listen for UDP messages on a specific port"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(1.0)
            sock.bind(('0.0.0.0', port))
            
            self.print_status(f"Listening on UDP port {port} for {duration} seconds")
            
            start_time = time.time()
            messages_received = 0
            
            while time.time() - start_time < duration:
                try:
                    data, addr = sock.recvfrom(1024)
                    messages_received += 1
                    self.print_status(f"Port {port}: Received '{data.decode('utf-8', errors='ignore')}' from {addr}", "PASS")
                except socket.timeout:
                    continue
                except Exception as e:
                    self.print_status(f"Port {port}: Error receiving data: {e}", "FAIL")
                    break
            
            sock.close()
            
            if messages_received > 0:
                self.print_status(f"Port {port}: Received {messages_received} messages", "PASS")
                return True
            else:
                self.print_status(f"Port {port}: No messages received", "FAIL")
                return False
                
        except Exception as e:
            self.print_status(f"Port {port}: Failed to bind/listen: {e}", "FAIL")
            return False
    
    def send_to_port(self, target_ip, port, message="DDS_TEST", count=5):
        """Send UDP messages to a specific port"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            success_count = 0
            for i in range(count):
                test_message = f"{message}_{i+1}_{datetime.now().strftime('%H%M%S')}"
                
                try:
                    sock.sendto(test_message.encode(), (target_ip, port))
                    success_count += 1
                    self.print_status(f"Port {port}: Sent '{test_message}' to {target_ip}:{port}")
                    time.sleep(0.5)
                except Exception as e:
                    self.print_status(f"Port {port}: Failed to send to {target_ip}:{port}: {e}", "FAIL")
            
            sock.close()
            
            if success_count == count:
                self.print_status(f"Port {port}: Successfully sent {success_count}/{count} messages", "PASS")
                return True
            else:
                self.print_status(f"Port {port}: Only sent {success_count}/{count} messages", "WARN")
                return False
                
        except Exception as e:
            self.print_status(f"Port {port}: Send operation failed: {e}", "FAIL")
            return False
    
    def test_bidirectional_connectivity(self, remote_ip, test_port=7400):
        """Test bidirectional connectivity on a specific port"""
        self.print_status(f"=== Testing bidirectional connectivity with {remote_ip} on port {test_port} ===")
        
        # Start listener in a separate thread
        listen_thread = threading.Thread(
            target=self.listen_on_port, 
            args=(test_port, 10)
        )
        listen_thread.daemon = True
        listen_thread.start()
        
        # Give listener time to start
        time.sleep(1)
        
        # Send test messages
        send_success = self.send_to_port(remote_ip, test_port, "CONNECTIVITY_TEST", 3)
        
        # Wait for listener to complete
        listen_thread.join(timeout=12)
        
        return send_success
    
    def scan_dds_ports(self):
        """Scan for activity on all DDS-related ports"""
        self.print_status("=== Scanning DDS ports for activity ===")
        
        active_ports = []
        
        for port in DDS_PORTS:
            try:
                # Quick check if port is in use
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(0.1)
                result = sock.connect_ex(('127.0.0.1', port))
                sock.close()
                
                # Try to bind briefly to see if port is free
                test_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                test_sock.settimeout(0.1)
                try:
                    test_sock.bind(('127.0.0.1', port))
                    self.print_status(f"Port {port}: Available (no activity)", "INFO")
                    test_sock.close()
                except:
                    self.print_status(f"Port {port}: In use or restricted", "WARN")
                    active_ports.append(port)
                    test_sock.close()
                    
            except Exception as e:
                self.print_status(f"Port {port}: Error checking: {e}", "WARN")
        
        if active_ports:
            self.print_status(f"Active DDS ports detected: {active_ports}", "INFO")
        else:
            self.print_status("No active DDS ports detected - ROS2 may not be running", "WARN")
        
        return active_ports
    
    def run_listener_mode(self, duration=30):
        """Run in listener mode for all DDS ports"""
        self.print_status(f"=== Listener Mode: Monitoring DDS ports for {duration} seconds ===")
        
        threads = []
        for port in DDS_PORTS:
            thread = threading.Thread(
                target=self.listen_on_port,
                args=(port, duration)
            )
            thread.daemon = True
            thread.start()
            threads.append(thread)
        
        # Wait for all listeners to complete
        for thread in threads:
            thread.join()
    
    def run_sender_mode(self, target_ip):
        """Run in sender mode to all DDS ports"""
        self.print_status(f"=== Sender Mode: Testing connectivity to {target_ip} ===")
        
        results = {}
        for port in DDS_PORTS:
            success = self.send_to_port(target_ip, port, f"DDS_PORT_{port}_TEST", 3)
            results[port] = success
            time.sleep(1)  # Brief pause between port tests
        
        # Summary
        successful_ports = [port for port, success in results.items() if success]
        failed_ports = [port for port, success in results.items() if not success]
        
        self.print_status(f"Summary: {len(successful_ports)}/{len(DDS_PORTS)} ports successful", 
                         "PASS" if len(successful_ports) > 0 else "FAIL")
        
        if successful_ports:
            self.print_status(f"Successful ports: {successful_ports}", "PASS")
        if failed_ports:
            self.print_status(f"Failed ports: {failed_ports}", "FAIL")
        
        return results

def main():
    parser = argparse.ArgumentParser(description="UDP Connectivity Test for ROS2 DDS")
    parser.add_argument('--mode', choices=['listen', 'send', 'scan', 'bidirectional'], 
                       default='scan', help='Test mode to run')
    parser.add_argument('--target-ip', help='Target IP address for send/bidirectional mode')
    parser.add_argument('--duration', type=int, default=30, 
                       help='Duration for listen mode (seconds)')
    parser.add_argument('--port', type=int, default=7400,
                       help='Specific port for bidirectional test')
    
    args = parser.parse_args()
    
    tester = UDPConnectivityTester()
    
    tester.print_status("=== ROS2 DDS UDP Connectivity Tester ===")
    tester.print_status(f"Mode: {args.mode}")
    
    if args.mode == 'scan':
        tester.scan_dds_ports()
        
    elif args.mode == 'listen':
        tester.run_listener_mode(args.duration)
        
    elif args.mode == 'send':
        if not args.target_ip:
            tester.print_status("Error: --target-ip required for send mode", "FAIL")
            sys.exit(1)
        tester.run_sender_mode(args.target_ip)
        
    elif args.mode == 'bidirectional':
        if not args.target_ip:
            tester.print_status("Error: --target-ip required for bidirectional mode", "FAIL")
            sys.exit(1)
        tester.test_bidirectional_connectivity(args.target_ip, args.port)
    
    tester.print_status("Test completed")

if __name__ == "__main__":
    main()