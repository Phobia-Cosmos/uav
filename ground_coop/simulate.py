#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Local Simulation Test

本地模拟测试 - 在一台机器上模拟三个节点的通信
使用127.0.0.1的不同端口模拟不同节点
"""

import sys
import time
import threading
import signal
import random
from pathlib import Path

# 添加项目根目录
PROJECT_ROOT = Path(__file__).resolve().parent
# TODO：这个insert的参数都是什么意思？这里加入的环境变量是永久的还是暂时的？
sys.path.insert(0, str(PROJECT_ROOT))

from common.protocol import (
    MessageType, Message, create_message,
    msg_status, msg_move_done, msg_heartbeat_ack, msg_error
)
from common.tcp_base import TCPClient, TCPServer, TCPConnection


# 模拟配置
SIMULATION_CONFIG = {
    "pc": {"ip": "127.0.0.1", "ports": {"from_drone": 15100, "from_dog": 15101}},
    "drone": {"ip": "127.0.0.1", "ports": {"from_pc": 15200, "to_dog": 15300}},
    "dog": {"ip": "127.0.0.1", "port": 15400},
    "heartbeat": {"interval": 10, "timeout": 30},
    "flight": {"default_altitude": 3.0},
    "control": {"move_interval": 5, "move_angle_min": 0, "move_angle_max": 360}
}


class SimulatedDog:
    """模拟机器狗"""
    
    def __init__(self, port):
        self.port = port
        self.logger = print
        self.position = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.running = False
        self.server = None
        self.drone_conn = None
    
    def start(self):
        self.running = True
        self.server = TCPServer(port=self.port)
        self.server.set_callbacks(
            on_client_connected=self._on_connected,
            on_data=self._on_data
        )
        self.server.start()
        print(f"[Dog] Started on port {self.port}")
    
    def stop(self):
        self.running = False
        if self.server:
            self.server.stop()
        print("[Dog] Stopped")
    
    def _on_connected(self, conn):
        self.drone_conn = conn
        print(f"[Dog] Drone connected: {conn.address}")
    
    def _on_data(self, conn, data):
        try:
            text = data.decode('utf-8')
            msg = Message.from_json(text)
            
            print(f"[Dog] Received: {msg.type}")
            
            if msg.type == "move":
                angle = msg.payload.get("angle", 0)
                duration = msg.payload.get("duration", 5)
                print(f"[Dog] Moving: angle={angle}°, duration={duration}s")
                
                # 模拟移动
                time.sleep(0.5)
                self.position["x"] += random.uniform(-1, 1)
                self.position["y"] += random.uniform(-1, 1)
                self.position["yaw"] = angle
                
                response = msg_move_done(success=True, position=self.position)
                conn.send(response.to_json().encode())
                print(f"[Dog] Move done: {self.position}")
            
            elif msg.type == "return":
                print("[Dog] Returning home...")
                time.sleep(0.3)
                self.position = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                response = msg_move_done(success=True, position=self.position)
                conn.send(response.to_json().encode())
                print("[Dog] Returned home")
            
            elif msg.type == "heartbeat":
                conn.send(msg_heartbeat_ack().to_json().encode())
        
        except Exception as e:
            print(f"[Dog] Error: {e}")


class SimulatedDrone:
    """模拟无人机"""
    
    def __init__(self, pc_port, dog_port):
        self.pc_port = pc_port
        self.dog_port = dog_port
        self.logger = print
        self.position = {"lat": 30.0, "lon": 120.0, "alt": 0.0}
        self.state = "disarmed"
        self.running = False
        self.pc_server = None
        self.dog_client = None
        self.pc_conn = None
        self.control_mode = False
    
    def start(self):
        self.running = True
        
        # 启动PC连接监听
        self.pc_server = TCPServer(port=self.pc_port)
        self.pc_server.set_callbacks(
            on_client_connected=self._on_pc_connected,
            on_data=self._on_pc_data
        )
        self.pc_server.start()
        print(f"[Drone] PC server started on port {self.pc_port}")
        
        # 连接机器狗
        time.sleep(0.5)
        self.dog_client = TCPClient("127.0.0.1", self.dog_port)
        self.dog_client.set_callbacks(
            on_connected=self._on_dog_connected,
            on_data=self._on_dog_data
        )
        if self.dog_client.connect():
            print(f"[Drone] Connected to dog on port {self.dog_port}")
        
        print("[Drone] Started")
    
    def stop(self):
        self.running = False
        self.control_mode = False
        if self.pc_server:
            self.pc_server.stop()
        if self.dog_client:
            self.dog_client.close()
        print("[Drone] Stopped")
    
    def _on_pc_connected(self, conn):
        self.pc_conn = conn
        print(f"[Drone] PC connected: {conn.address}")
    
    def _on_pc_data(self, conn, data):
        try:
            text = data.decode('utf-8')
            msg = Message.from_json(text)
            
            print(f"[Drone] From PC: {msg.type}")
            
            if msg.type == "start":
                alt = msg.payload.get("altitude", 3.0)
                self.state = "taking_off"
                time.sleep(0.5)
                self.state = "hovering"
                self.position["alt"] = alt
                print(f"[Drone] Took off to {alt}m")
                self._send_status("hovering")
            
            elif msg.type == "stop":
                self.control_mode = False
                print("[Drone] Stopping...")
                if self.dog_client and self.dog_client.is_connected:
                    # 先让机器狗返航
                    response = self.dog_client.send_text(create_message("return").to_json())
                self.state = "returning"
                time.sleep(0.5)
                self.state = "landing"
                time.sleep(0.5)
                self.state = "landed"
                self.position["alt"] = 0
                self._send_status("landed")
            
            elif msg.type == "control":
                print("[Drone] Entering control mode")
                self.control_mode = True
                self._control_loop()
            
            elif msg.type == "heartbeat":
                conn.send(msg_heartbeat_ack().to_json().encode())
        
        except Exception as e:
            print(f"[Drone] Error: {e}")
    
    def _on_dog_connected(self):
        print("[Drone] Dog connected")
    
    def _on_dog_data(self, data):
        try:
            text = data.decode('utf-8')
            msg = Message.from_json(text)
            
            if msg.type == "move_done":
                print(f"[Drone] Dog move done: {msg.payload}")
                self._forward_dog_result(msg.payload)
            
            elif msg.type == "heartbeat_ack":
                pass
        
        except Exception as e:
            print(f"[Drone] Dog data error: {e}")
    
    def _control_loop(self):
        """控制循环"""
        move_interval = SIMULATION_CONFIG["control"]["move_interval"]
        
        while self.control_mode and self.running:
            # 生成随机移动
            angle = random.uniform(0, 360)
            duration = random.uniform(1, 5)
            
            print(f"[Drone] Sending move to dog: angle={angle}, duration={duration}")
            
            if self.dog_client and self.dog_client.is_connected:
                move_msg = create_message("move", {"angle": angle, "duration": duration})
                response = self.dog_client.send_text(move_msg.to_json())
                if response:
                    resp_msg = Message.from_json(response.decode())
                    if resp_msg:
                        self._forward_dog_result(resp_msg.payload)
            
            # 等待
            for _ in range(move_interval):
                if not self.control_mode:
                    break
                time.sleep(0.5)
    
    def _send_status(self, state, extra=None):
        if self.pc_conn:
            status = msg_status(
                state=state,
                altitude=self.position.get("alt", 0),
                battery=100,
                position={"lat": self.position.get("lat", 0), "lon": self.position.get("lon", 0)},
                extra=extra
            )
            self.pc_conn.send(status.to_json().encode())
    
    def _forward_dog_result(self, result):
        if self.pc_conn:
            status = msg_status(
                state="moving",
                extra={"dog_result": result}
            )
            self.pc_conn.send(status.to_json().encode())


def run_simulation():
    """运行模拟"""
    print("=" * 60)
    print("   Local Simulation Test - Ground-Air Cooperation")
    print("=" * 60)
    
    # 创建模拟节点
    dog = SimulatedDog(SIMULATION_CONFIG["dog"]["port"])
    drone = SimulatedDrone(
        SIMULATION_CONFIG["drone"]["ports"]["from_pc"],
        SIMULATION_CONFIG["drone"]["ports"]["to_dog"]
    )
    
    # 启动线程
    dog_thread = threading.Thread(target=dog.start, daemon=True)
    drone_thread = threading.Thread(target=drone.start, daemon=True)
    
    dog_thread.start()
    drone_thread.start()
    
    # 等待连接稳定
    time.sleep(1.5)
    
    # 创建PC客户端
    print("\n" + "-" * 60)
    print("   Commands: start [alt], stop, control, status, quit")
    print("-" * 60)
    
    # 连接无人机
    drone_client = TCPClient("127.0.0.1", SIMULATION_CONFIG["drone"]["ports"]["from_pc"])
    
    if drone_client.connect():
        print("[PC] Connected to drone")
        
        try:
            while True:
                cmd = input("\n[PC] > ").strip().lower()
                
                if not cmd:
                    continue
                
                parts = cmd.split()
                action = parts[0]
                
                if action == "start":
                    alt = float(parts[1]) if len(parts) > 1 else 3.0
                    msg = create_message("start", {"altitude": alt})
                    drone_client.send_text(msg.to_json())
                    print(f"[PC] Sent START: alt={alt}m")
                
                elif action == "stop":
                    msg = create_message("stop")
                    drone_client.send_text(msg.to_json())
                    print("[PC] Sent STOP")
                
                elif action == "control":
                    msg = create_message("control")
                    drone_client.send_text(msg.to_json())
                    print("[PC] Sent CONTROL - Watch the dog move!")
                
                elif action == "status":
                    print("[PC] Status check... (wait for drone response)")
                
                elif action == "quit":
                    print("[PC] Quitting...")
                    break
                
                else:
                    print("[PC] Unknown command. Use: start, stop, control, status, quit")
                
                time.sleep(0.3)
                
        except (EOFError, KeyboardInterrupt):
            print("\n[PC] Interrupted")
        
        drone_client.close()
    else:
        print("[PC] Failed to connect to drone")
    
    # 停止
    dog.stop()
    drone.stop()
    
    print("\nSimulation ended")


if __name__ == "__main__":
    run_simulation()
