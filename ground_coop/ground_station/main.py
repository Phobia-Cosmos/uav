#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ground Station Main Program

PC地面站主程序，用户交互界面。
"""

import sys
import os
import time
import signal
from pathlib import Path
from typing import Optional

# 确保common模块可导入
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from common import (
    get_logger, get_config, parse_message,
    msg_start, msg_stop, msg_control, msg_heartbeat, msg_test,
    TCPClient, TCPConnection,
    HeartbeatManager, HeartbeatCallback
)


class GroundStation:
    """地面站"""
    
    def __init__(self, config):
        self.config = config
        self.logger = get_logger("ground_station")
        
        self.drone_port = config.get_port("from_drone", "pc")
        self.dog_port = config.get_port("from_dog", "pc")
        self.drone_ip = config.get_ip("drone")
        self.drone_connect_port = 5200  # PC 连接到无人机的端口
        
        self.heartbeat_config = config.get_heartbeat_config()
        
        self.drone_server = None
        self.dog_server = None
        self.drone_client = None
        self.drone_connection = None
        self.dog_connection = None
        
        self.running = False
        self.last_drone_status = None
        self.last_dog_status = None
        
        self.heartbeat = HeartbeatManager(
            interval=self.heartbeat_config.get("interval", 10),
            timeout=self.heartbeat_config.get("timeout", 30),
            callback=HeartbeatCallback(
                on_send=self._send_heartbeats,
                on_timeout=self._on_heartbeat_timeout
            )
        )
        
        self.logger.info("Ground station initialized")
    
    def _on_heartbeat_timeout(self):
        self.logger.warning("Heartbeat timeout from drone")
        print(f"\n[Warning: Drone heartbeat lost]")
    
    def _send_heartbeats(self):
        if self.drone_client and self.drone_client.is_connected:
            try:
                self.drone_client.send(msg_heartbeat().to_json().encode())
            except:
                pass
    
    def start(self):
        self.logger.info("Starting ground station in monitor mode...")
        self.logger.info(f"Drone: {self.drone_ip}:{self.drone_connect_port}")
        self.logger.info("Waiting for drone connection...")
        self.logger.info("Use 'connect' to connect, 'quit' to exit")
        
        self.running = True
        self.heartbeat.start()
        
        self.logger.info(f"Ground station started")
        return True
    
    def connect_to_drone(self):
        """连接到无人机"""
        print(f"\nConnecting to {self.drone_ip}:{self.drone_connect_port}...")
        
        if self.drone_client:
            self.drone_client.disconnect()
        
        self.drone_client = TCPClient(host=self.drone_ip, port=self.drone_connect_port)
        self.drone_client.set_callbacks(
            on_connected=self._on_drone_connected,
            on_disconnected=self._on_drone_disconnected,
            on_data=self._on_drone_data
        )
        
        if self.drone_client.connect():
            print(f"Connected to drone!")
            return True
        else:
            print(f"Failed to connect. Make sure drone is running on {self.drone_ip}:{self.drone_connect_port}")
            print("Use 'quit' to exit, or wait and try 'connect' again.")
            return False
    
    def stop(self):
        self.logger.info("Stopping ground station...")
        self.running = False
        self.heartbeat.stop()
        
        if self.drone_client:
            self.drone_client.disconnect()
        
        self.logger.info("Ground station stopped")
        self.logger.info("Stopping ground station...")
        self.running = False
        self.heartbeat.stop()
        
        if self.drone_client:
            self.drone_client.disconnect()
        
        self.logger.info("Ground station stopped")
    
    def _on_drone_connected(self):
        self.logger.info(f"Drone connected: {self.drone_ip}:{self.drone_connect_port}")
        print(f"\n[Drone connected!]")
        self.heartbeat.mark_received()
    
    def _on_drone_disconnected(self):
        self.logger.info(f"Drone disconnected")
        print(f"\n[Drone disconnected]")
        self.heartbeat.reset()
    
    def _on_dog_connected(self, conn: TCPConnection):
        self.logger.info(f"Dog connected: {conn.address}")
        self.dog_connection = conn
    
    def _on_drone_data(self, data: bytes):
        try:
            text = data.decode('utf-8')
            msg = parse_message(text)
            
            if msg:
                self.last_drone_status = msg.payload
                self.heartbeat.mark_received()
                
                if msg.type == "status":
                    self._display_status("Drone", msg.payload)
                elif msg.type == "error":
                    self._display_error("Drone", msg.payload)
                elif msg.type == "heartbeat_ack":
                    pass
                    
        except Exception as e:
            self.logger.error(f"Error handling drone data: {e}")
    
    def _on_dog_data(self, conn: TCPConnection, data: bytes):
        try:
            text = data.decode('utf-8')
            msg = parse_message(text)
            
            if msg:
                self.last_dog_status = msg.payload
                
                if msg.type == "status":
                    self._display_status("Dog", msg.payload)
                elif msg.type == "move_done":
                    self._display_dog_result(msg.payload)
                    
        except Exception as e:
            self.logger.error(f"Error handling dog data: {e}")
    
    def _display_status(self, device: str, status: dict):
        state = status.get("state", "unknown")
        alt = status.get("altitude", 0)
        battery = status.get("battery", 0)
        pitch = status.get("pitch", 0)
        roll = status.get("roll", 0)
        yaw = status.get("yaw", 0)
        
        print(f"\n[{device}] State: {state}")
        print(f"         Alt: {alt:.1f}m | Battery: {battery}%")
        print(f"         Pitch: {pitch:.1f}° | Roll: {roll:.1f}° | Yaw: {yaw:.1f}°")
    
    def _display_error(self, device: str, error: dict):
        print(f"\n[{device}] ERROR: {error.get('code')}: {error.get('message')}")
    
    def _display_dog_result(self, result: dict):
        success = result.get("success", False)
        pos = result.get("position", {})
        print(f"\n[Dog] Move {'SUCCESS' if success else 'FAILED'} | Pos: {pos}")
    
    def send_to_drone(self, msg):
        if self.drone_client and self.drone_client.is_connected:
            try:
                self.drone_client.send(msg.to_json().encode())
                return True
            except Exception as e:
                self.logger.error(f"Failed to send to drone: {e}")
        else:
            self.logger.warning("Drone not connected")
        return False
    
    def send_start(self, altitude: float = 3.0, mode: str = "GUIDED"):
        self.logger.info(f"Sending START command: altitude={altitude}m, mode={mode}")
        return self.send_to_drone(msg_start(altitude, mode))
    
    def send_stop(self):
        self.logger.info("Sending STOP command")
        return self.send_to_drone(msg_stop())
    
    def send_control(self):
        self.logger.info("Sending CONTROL command")
        return self.send_to_drone(msg_control())
    
    def send_test(self, filename: str):
        self.logger.info(f"Sending TEST command: {filename}")
        return self.send_to_drone(msg_test(filename))


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Ground Station")
    parser.add_argument("--config", type=str, default=None)
    parser.add_argument("--auto", action="store_true")
    parser.add_argument("--wifi", type=str)
    parser.add_argument("--log-level", type=str, default="INFO")
    
    args = parser.parse_args()
    
    logger = get_logger("ground_station", level=args.log_level)
    logger.info("=== Ground Station Starting ===")
    
    config = get_config(args.config)
    
    if args.wifi:
        config.detect_wifi()
    
    logger.info(f"Drone IP: {config.get_ip('drone')}")
    
    station = GroundStation(config)
    
    def signal_handler(sig, frame):
        logger.info("Received shutdown signal")
        station.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    if not station.start():
        logger.error("Failed to start ground station")
        return
    
    print("\n" + "=" * 50)
    print("   Ground-Air Cooperation System")
    print("=" * 50)
    print("\nDrone not connected yet.")
    print("Commands: connect, help, quit")
    print("-" * 50)
    
    while station.running:
        try:
            cmd = input("\n> ").strip().lower()
            
            if not cmd:
                continue
            
            parts = cmd.split()
            action = parts[0]
            
            if action == "connect":
                station.connect_to_drone()
            elif action == "help":
                print("\n=== Commands ===")
                print("  connect              - Connect to drone")
                print("  disconnect           - Disconnect from drone")
                print("  start [alt] [mode]   - Takeoff (alt: meters, mode: GUIDED/STABILIZE/HOLD/ALT_HOLD)")
                print("  stop                 - Land and disarm")
                print("  control              - Enter control mode (drone controls dog)")
                print("  status               - Show drone status")
                print("  test <filename>      - Run test script on drone (absolute path or relative)")
                print("  quit                 - Exit program")
                print("")
    
    while station.running:
        try:
            cmd = input("\n> ").strip().lower()
            
            if not cmd:
                continue
            
            parts = cmd.split()
            action = parts[0]
            
            if action == "connect":
                station.connect_to_drone()
            elif action == "disconnect":
                if station.drone_client and station.drone_client.is_connected:
                    station.drone_client.disconnect()
                    print("\nDisconnected from drone. Use 'connect' to reconnect.")
                else:
                    print("\nNot connected to drone.")
            elif action == "start":
                if not (station.drone_client and station.drone_client.is_connected):
                    print("\nNot connected to drone. Use 'connect' first.")
                else:
                    alt = float(parts[1]) if len(parts) > 1 else 3.0
                    mode = parts[2].upper() if len(parts) > 2 else "GUIDED"
                    station.send_start(alt, mode)
            elif action == "stop":
                if station.drone_client and station.drone_client.is_connected:
                    station.send_stop()
                else:
                    print("\nNot connected to drone.")
            elif action == "control":
                if station.drone_client and station.drone_client.is_connected:
                    station.send_control()
                else:
                    print("\nNot connected to drone.")
            elif action == "status":
                if station.drone_client and station.drone_client.is_connected:
                    if station.last_drone_status:
                        station._display_status("Drone", station.last_drone_status)
                    else:
                        print("\nWaiting for drone status...")
                else:
                    print("\nNot connected to drone. Use 'connect' to connect.")
            elif action == "test":
                if not (station.drone_client and station.drone_client.is_connected):
                    print("\nNot connected to drone. Use 'connect' first.")
                else:
                    if len(parts) < 2:
                        print("\nUsage: test <filename>")
                        print("  Example: test /home/orangepi/Desktop/uav/noGPS")
                        print("  Example: test noGPS (relative to ground_coop dir)")
                    else:
                        test_file = parts[1]
                        station.send_test(test_file)
                        print(f"\nRunning test: {test_file}")
            elif action == "help":
                print("\n=== Commands ===")
                print("  connect              - Connect to drone")
                print("  disconnect           - Disconnect from drone")
                print("  start [alt] [mode]   - Takeoff (alt: meters, mode: GUIDED/STABILIZE/HOLD/ALT_HOLD)")
                print("  stop                 - Land and disarm")
                print("  control              - Enter control mode (drone controls dog)")
                print("  status               - Show drone status")
                print("  test <filename>      - Run test script on drone (absolute path or relative to ground_coop)")
                print("  quit                 - Exit program")
                print("")
            elif action == "quit":
                print("Quitting...")
                break
            else:
                print("Unknown command. Use: connect, start [alt] [mode], stop, control, status, disconnect, quit")
                
        except EOFError:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    station.stop()
    logger.info("=== Ground Station Stopped ===")


if __name__ == "__main__":
    main()
