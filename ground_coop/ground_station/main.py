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
    msg_start, msg_stop, msg_control, msg_heartbeat,
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
        self.logger.warning("Heartbeat timeout from devices")
    
    def _send_heartbeats(self):
        if self.drone_client and self.drone_client.is_connected:
            try:
                self.drone_client.send(msg_heartbeat().to_json().encode())
            except:
                pass
    
    def stop(self):
        self.logger.info("Stopping ground station...")
        self.running = False
        self.heartbeat.stop()
        
        if self.drone_client:
            self.drone_client.disconnect()
        
        self.logger.info("Ground station stopped")
    
    def _on_drone_connected(self):
        self.logger.info(f"Drone connected: {self.drone_ip}:{self.drone_port}")
        self.heartbeat.mark_received()
    
    def _on_drone_disconnected(self):
        self.logger.info(f"Drone disconnected")
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
        
        print(f"\n[{device}] State: {state} | Alt: {alt:.1f}m | Battery: {battery}%")
    
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
    
    def send_start(self, altitude: float = 3.0):
        self.logger.info(f"Sending START command: altitude={altitude}m")
        return self.send_to_drone(msg_start(altitude))
    
    def send_stop(self):
        self.logger.info("Sending STOP command")
        return self.send_to_drone(msg_stop())
    
    def send_control(self):
        self.logger.info("Sending CONTROL command")
        return self.send_to_drone(msg_control())


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
    print("\nWaiting for connections...")
    print("Commands: start [altitude], stop, control, status, quit")
    print("-" * 50)
    
    while station.running:
        try:
            cmd = input("\n> ").strip().lower()
            
            if not cmd:
                continue
            
            parts = cmd.split()
            action = parts[0]
            
            if action == "start":
                alt = float(parts[1]) if len(parts) > 1 else 3.0
                station.send_start(alt)
            elif action == "stop":
                station.send_stop()
            elif action == "control":
                station.send_control()
            elif action == "status":
                if station.last_drone_status:
                    station._display_status("Drone", station.last_drone_status)
                if station.last_dog_status:
                    station._display_status("Dog", station.last_dog_status)
            elif action == "quit":
                print("Quitting...")
                break
            else:
                print("Unknown command. Use: start [alt], stop, control, status, quit")
                
        except EOFError:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    station.stop()
    logger.info("=== Ground Station Stopped ===")


if __name__ == "__main__":
    main()
