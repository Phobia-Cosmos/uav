#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Drone Main Program

无人机端主程序，监听PC命令并控制飞控和机器狗。
"""

import sys
import time
import signal
import threading
from pathlib import Path
from typing import Optional

# 确保common模块可导入
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from common import (
    get_logger, get_config, parse_message,
    msg_status, msg_heartbeat_ack, msg_error,
    TCPClient, TCPServer, TCPConnection,
    HeartbeatManager, HeartbeatCallback
)
from drone import FlightController, DogCommander, FlightState
import subprocess


class DroneServer:
    """无人机服务端"""
    
    def __init__(self, config):
        self.config = config
        self.logger = get_logger("drone")
        
        self.pc_port = config.get_port("from_pc", "drone")
        self.dog_ip = config.get_ip("dog")
        self.dog_port = config.get_port("to_dog", "drone")
        
        self.heartbeat_config = config.get_heartbeat_config()
        self.flight_config = config.get_flight_config()
        self.control_config = config.get_control_config()
        
        self.pc_server = None
        self.pc_connection = None
        
        self.flight = FlightController()
        self.dog_commander = None
        
        self.running = False
        self.control_mode = False
        self.last_dog_result = None
        
        self.heartbeat = HeartbeatManager(
            interval=self.heartbeat_config.get("interval", 10),
            timeout=self.heartbeat_config.get("timeout", 30),
            callback=HeartbeatCallback(
                on_timeout=self._on_heartbeat_timeout
            )
        )
        
        self.logger.info("Drone server initialized")
    
    def _on_heartbeat_timeout(self):
        self.logger.warning("PC heartbeat timeout")
        if self.control_mode:
            self.logger.info("Switching to return mode")
            self._handle_stop()
    
    def start(self):
        self.logger.info(f"Starting drone server on port {self.pc_port}...")
        
        self.flight.set_logger(self.logger)
        
        self.pc_server = TCPServer(port=self.pc_port)
        self.pc_server.set_callbacks(
            on_client_connected=self._on_pc_connected,
            on_client_disconnected=self._on_pc_disconnected,
            on_data=self._on_pc_data
        )
        
        if not self.pc_server.start():
            self.logger.error("Failed to start PC server")
            return False
        
        self.running = True
        self.heartbeat.start()
        
        self.logger.info(f"Drone server started on port {self.pc_port}")
        return True
    
    def stop(self):
        self.logger.info("Stopping drone server...")
        self.running = False
        self.control_mode = False
        self.heartbeat.stop()
        
        if self.pc_server:
            self.pc_server.stop()
        
        if self.flight.vehicle:
            self.logger.info("Cleaning up flight controller...")
            
            try:
                from dronekit import VehicleMode
                
                if self.flight.state not in [FlightState.DISARMED, FlightState.LANDED]:
                    self.logger.info("Landing...")
                    self.flight.land()
                    time.sleep(2)
                
                if self.flight.vehicle.armed:
                    self.logger.info("Disarming motors...")
                    self.flight.disarm()
                
            except Exception as e:
                self.logger.error(f"Flight cleanup error: {e}")
        
        self.flight.disconnect()
        
        if self.dog_commander:
            self.dog_commander.disconnect()
        
        self.logger.info("Drone server stopped")
    
    def _on_pc_connected(self, conn: TCPConnection):
        self.logger.info(f"PC connected: {conn.address}")
        self.pc_connection = conn
        self.heartbeat.mark_received()
    
    def _on_pc_disconnected(self, conn: TCPConnection):
        self.logger.info(f"PC disconnected: {conn.address}")
        self.pc_connection = None
        self.heartbeat.reset()
    
    def _on_pc_data(self, conn: TCPConnection, data: bytes):
        try:
            text = data.decode('utf-8')
            msg = parse_message(text)
            
            if msg is None:
                return
            
            self.heartbeat.mark_received()
            if msg.type != "heartbeat":
                self.logger.info(f"Received from PC: {msg.type}")
            self._handle_message(msg)
            
        except Exception as e:
            self.logger.error(f"Error handling PC message: {e}")
    
    def _handle_message(self, msg):
        msg_type = msg.type
        
        if msg_type == "start":
            self._handle_start(msg)
        elif msg_type == "stop":
            self._handle_stop()
        elif msg_type == "control":
            self._handle_control(msg)
        elif msg_type == "heartbeat":
            self._handle_heartbeat(msg)
        elif msg_type == "test":
            self._handle_test(msg)
        else:
            self.logger.warning(f"Unknown message: {msg_type}")
    
    def _handle_start(self, msg):
        altitude = msg.payload.get("altitude", self.flight_config.get("default_altitude", 3.0))
        mode = msg.payload.get("mode", "GUIDED").upper()
        
        self.logger.info(f"Starting: altitude={altitude}m, mode={mode}")
        
        if not self.flight.vehicle:
            if not self.flight.connect():
                self._send_error("FLIGHT_ERROR", "Failed to connect to flight controller")
                return
        
        if self.flight.state != FlightState.DISARMED:
            self._send_status(state=self.flight.state.value)
            return
        
        if mode in ["STABILIZE", "HOLD", "ALT_HOLD"]:
            self.logger.info(f"Mode {mode}: skipping GPS check for indoor flight")
            if self.flight.takeoff(altitude, require_gps=False):
                self._send_status(state="hovering")
            else:
                self._send_error("TAKEOFF_FAILED", "Takeoff failed")
        else:
            if self.flight.takeoff(altitude, require_gps=True):
                self._send_status(state="hovering")
            else:
                self._send_error("TAKEOFF_FAILED", "Takeoff failed")
    
    def _handle_stop(self):
        self.logger.info("Stopping...")
        self.control_mode = False
        
        if self.dog_commander and self.dog_commander.connected:
            result = self.dog_commander.return_home()
            self._forward_dog_result(result)
        
        if self.flight.state not in [FlightState.DISARMED, FlightState.LANDED]:
            self.flight.return_to_launch()
            self.flight.land()
            self.flight.disarm()
        
        self._send_status(state="landed")
    
    def _handle_control(self, msg):
        self.logger.info("Entering control mode")
        self.control_mode = True
        
        if not self.dog_commander:
            dog_config = self.config.get_device_config("dog")
            self.dog_commander = DogCommander(
                dog_ip=dog_config.get("ip", ""),
                dog_port=dog_config.get("port", 5400)
            )
        
        if not self.dog_commander.connected:
            if not self.dog_commander.connect():
                self._send_error("DOG_CONNECTION_FAILED", "Failed to connect to dog")
                self.control_mode = False
                return
        
        self._control_loop()
    
    def _control_loop(self):
        move_interval = self.control_config.get("move_interval", 10)
        
        while self.control_mode and self.running:
            self.flight.update_status()
            
            if not self.heartbeat.is_alive():
                self.logger.warning("PC heartbeat lost")
                break
            
            angle, duration = self.dog_commander.generate_random_move(self.control_config)
            
            self.logger.info(f"Moving dog: angle={angle}°, duration={duration}s")
            
            result = self.dog_commander.move(angle, duration)
            self.last_dog_result = result
            
            self._forward_dog_result(result)
            
            self.flight.hover(0.5)
            
            for _ in range(move_interval * 2):
                if not self.control_mode or not self.running:
                    break
                self.flight.update_status()
                self._send_status(state="moving")
                time.sleep(0.5)
    
    def _handle_heartbeat(self, msg):
        self.heartbeat.mark_received()
        self._send_response(msg_heartbeat_ack())
    
    def _handle_test(self, msg):
        filename = msg.payload.get("filename", "")
        
        if not filename:
            self.logger.warning("Test filename is empty")
            return
        
        if filename.endswith('.py'):
            script_path = filename
        elif filename.startswith('/'):
            script_path = f"{filename}.py"
        else:
            script_path = f"/home/orangepi/Desktop/uav/ground_coop/{filename}.py"
        
        self.logger.info(f"Running test: {script_path}")
        
        try:
            result = subprocess.run(
                ["python3", script_path],
                timeout=60,
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0:
                self.logger.info(f"Test completed successfully")
            else:
                self.logger.error(f"Test failed: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            self.logger.error(f"Test timed out")
        except Exception as e:
            self.logger.error(f"Test error: {e}")
    
    def _send_response(self, response):
        if self.pc_connection:
            try:
                self.pc_connection.send(response.to_json().encode('utf-8'))
            except Exception as e:
                self.logger.error(f"Failed to send response: {e}")
    
    def _send_status(self, state: str = "unknown", extra: dict = None):
        self.flight.update_status()
        current_state = state if state else (self.flight.state.value if self.flight.state else "unknown")
        current_extra = extra if extra else {}
        status = msg_status(
            state=current_state,
            altitude=self.flight.status.altitude,
            battery=self.flight.status.battery,
            position={
                "lat": self.flight.status.latitude,
                "lon": self.flight.status.longitude
            },
            extra={
                "pitch": round(self.flight.status.pitch, 1),
                "roll": round(self.flight.status.roll, 1),
                "yaw": round(self.flight.status.yaw, 1),
                **current_extra
            }
        )
        self._send_response(status)
    
    def _send_error(self, code: str, message: str):
        error = msg_error(code, message)
        self._send_response(error)
    
    def _forward_dog_result(self, result: dict):
        status = msg_status(
            state="moving",
            extra={"dog_result": result}
        )
        self._send_response(status)


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Drone Server")
    parser.add_argument("--config", type=str, default=None)
    parser.add_argument("--fc-connection", type=str, default="/dev/ttyACM0")
    parser.add_argument("--fc-baud", type=int, default=921600)
    parser.add_argument("--auto", action="store_true")
    parser.add_argument("--wifi", type=str)
    parser.add_argument("--log-level", type=str, default="INFO")
    
    args = parser.parse_args()
    
    logger = get_logger("drone", level=args.log_level)
    logger.info("=== Drone Server Starting ===")
    
    config = get_config(args.config)
    
    if args.wifi:
        config.detect_wifi()
    
    logger.info(f"Drone config: {config.get_device_config('drone')}")
    
    server = DroneServer(config)
    
    def signal_handler(sig, frame):
        logger.info("Received shutdown signal")
        server.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        if server.start():
            logger.info("Drone server is running. Press Ctrl+C to stop.")
            
            while server.running:
                time.sleep(1)
    except Exception as e:
        logger.error(f"Server error: {e}")
    finally:
        server.stop()
        logger.info("=== Drone Server Stopped ===")


if __name__ == "__main__":
    main()
