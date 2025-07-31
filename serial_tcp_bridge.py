#!/usr/bin/env python3
"""
Serial to TCP bridge for macOS Docker serial device access.
This script runs on the host macOS and creates a TCP server that forwards
serial data from the Arduino to any TCP clients (like the Docker container).
"""

import serial
import socket
import threading
import time
import sys

class SerialTCPBridge:
    def __init__(self, serial_port='/dev/cu.usbmodem11201', baud_rate=9600, tcp_port=8888):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.tcp_port = tcp_port
        self.running = False
        self.serial_conn = None
        self.clients = []

    def start_serial(self):
        """Initialize serial connection"""
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Allow Arduino to reset
            print(f"Serial connection established on {self.serial_port} at {self.baud_rate} baud")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to serial port {self.serial_port}: {e}")
            return False

    def start_tcp_server(self):
        """Start TCP server to accept connections from Docker container"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', self.tcp_port))
        self.server_socket.listen(5)
        print(f"TCP server listening on port {self.tcp_port}")

        while self.running:
            try:
                client_socket, address = self.server_socket.accept()
                print(f"Client connected from {address}")
                self.clients.append(client_socket)
                
                # Start a thread to handle this client
                client_thread = threading.Thread(target=self.handle_client, args=(client_socket,))
                client_thread.daemon = True
                client_thread.start()
                
            except socket.error:
                break

    def handle_client(self, client_socket):
        """Handle individual client connections"""
        try:
            while self.running:
                # Just keep the connection alive
                # The serial reading thread will send data to all clients
                time.sleep(1)
        except:
            pass
        finally:
            if client_socket in self.clients:
                self.clients.remove(client_socket)
            client_socket.close()

    def read_serial_and_broadcast(self):
        """Read from serial and broadcast to all TCP clients"""
        while self.running:
            if self.serial_conn and self.serial_conn.in_waiting > 0:
                try:
                    data = self.serial_conn.readline()
                    if data:
                        print(f"Serial data: {data.decode('utf-8', errors='ignore').strip()}")
                        
                        # Send to all connected clients
                        for client in self.clients[:]:  # Copy list to avoid modification during iteration
                            try:
                                client.send(data)
                            except:
                                # Remove dead clients
                                if client in self.clients:
                                    self.clients.remove(client)
                                client.close()
                                
                except serial.SerialException as e:
                    print(f"Serial error: {e}")
                    time.sleep(1)
            else:
                time.sleep(0.1)

    def start(self):
        """Start the bridge"""
        print("Starting Serial-TCP Bridge...")
        
        if not self.start_serial():
            print("Failed to initialize serial connection")
            return False

        self.running = True

        # Start TCP server in a separate thread
        tcp_thread = threading.Thread(target=self.start_tcp_server)
        tcp_thread.daemon = True
        tcp_thread.start()

        # Start serial reading in main thread
        try:
            self.read_serial_and_broadcast()
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.stop()

    def stop(self):
        """Stop the bridge"""
        self.running = False
        
        if self.serial_conn:
            self.serial_conn.close()
            
        for client in self.clients:
            client.close()
            
        if hasattr(self, 'server_socket'):
            self.server_socket.close()

if __name__ == "__main__":
    bridge = SerialTCPBridge()
    bridge.start()