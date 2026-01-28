#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dog Commander

无人机端与机器狗通信模块。
"""

import time
import random
from typing import Optional, Dict, Callable
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from common import (
    get_logger, TCPClient,
    msg_move, msg_move_done, msg_status, msg_return,
    parse_message, Message
)


class DogCommander:
    """机器狗指挥官"""
    
    def __init__(self, dog_ip: str, dog_port: int):
        """
        初始化
        
        Args:
            dog_ip: 机器狗IP地址
            dog_port: 机器狗监听端口
        """
        self.dog_ip = dog_ip
        self.dog_port = dog_port
        self.logger = get_logger("dog_commander")
        
        self.client: Optional[TCPClient] = None
        self.connected = False
        self.last_response = None
        self.response_received = False
        
        # 设置回调
        self.on_connected: Optional[Callable] = None
        self.on_disconnected: Optional[Callable] = None
        self.on_response: Optional[Callable[[Message], None]] = None
    
    def connect(self) -> bool:
        """连接机器狗"""
        self.logger.info(f"Connecting to dog: {self.dog_ip}:{self.dog_port}")
        
        self.client = TCPClient(self.dog_ip, self.dog_port)
        self.client.set_callbacks(
            on_connected=self._on_connected,
            on_disconnected=self._on_disconnected,
            on_data=self._on_data
        )
        
        if self.client.connect():
            self.connected = True
            self.logger.info("Connected to dog")
            return True
        
        self.logger.error("Failed to connect to dog")
        return False
    
    def disconnect(self):
        """断开连接"""
        if self.client:
            self.client.close()
        self.connected = False
        self.logger.info("Disconnected from dog")
    
    def _on_connected(self):
        """连接回调"""
        self.connected = True
        self.logger.info("Connected to dog")
        if self.on_connected:
            self.on_connected()
    
    def _on_disconnected(self):
        """断开回调"""
        self.connected = False
        self.logger.info("Disconnected from dog")
        if self.on_disconnected:
            self.on_disconnected()
    
    def _on_data(self, data: bytes):
        """数据接收回调"""
        try:
            text = data.decode('utf-8')
            msg = parse_message(text)
            
            if msg:
                self.last_response = msg
                self.response_received = True
                
                if self.on_response:
                    self.on_response(msg)
                    
        except Exception as e:
            self.logger.error(f"Error parsing response: {e}")
    
    def send_command(self, msg: Message, timeout: float = 10.0) -> Optional[Message]:
        """
        发送命令并等待响应
        
        Args:
            msg: 命令消息
            timeout: 超时时间
            
        Returns:
            响应消息
        """
        if not self.connected or not self.client:
            self.logger.error("Not connected to dog")
            return None
        
        self.response_received = False
        self.last_response = None
        
        # 发送命令
        if not self.client.send_text(msg.to_json()):
            self.logger.error("Failed to send command")
            return None
        
        self.logger.info(f"Sent command: {msg.type}")
        
        # 等待响应
        start = time.time()
        while time.time() - start < timeout:
            if self.response_received:
                return self.last_response
            time.sleep(0.1)
        
        self.logger.warning("Response timeout")
        return None
    
    def move(self, angle: float, duration: float) -> Dict:
        """
        发送移动命令
        
        Args:
            angle: 移动角度 (0-360)
            duration: 移动时间 (0-10)
            
        Returns:
            {"success": bool, "position": dict}
        """
        msg = msg_move(angle, duration)
        response = self.send_command(msg)
        
        if response and response.type == "move_done":
            return {
                "success": response.payload.get("success", False),
                "position": response.payload.get("position", {})
            }
        
        return {"success": False, "error": "No response"}
    
    def return_home(self) -> Dict:
        """
        发送返航命令
        
        Returns:
            {"success": bool, "position": dict}
        """
        msg = msg_return()
        response = self.send_command(msg)
        
        if response and response.type == "move_done":
            return {
                "success": response.payload.get("success", False),
                "position": response.payload.get("position", {})
            }
        
        return {"success": False, "error": "No response"}
    
    def stop(self) -> Dict:
        """发送停止命令"""
        msg = parse_message('{"msg_type": "stop"}')
        response = self.send_command(msg)
        
        if response:
            return {"success": True}
        
        return {"success": False, "error": "No response"}
    
    def get_status(self) -> Dict:
        """获取机器狗状态"""
        msg = parse_message('{"msg_type": "status"}')
        response = self.send_command(msg)
        
        if response:
            return {
                "state": response.payload.get("state", "unknown"),
                "position": response.payload.get("position", {})
            }
        
        return {"state": "unknown", "position": {}}
    
    def generate_random_move(self, config: Dict) -> tuple:
        """
        生成随机移动参数
        
        Args:
            config: 控制配置
            
        Returns:
            (angle, duration)
        """
        angle = random.uniform(
            config.get("move_angle_min", 0),
            config.get("move_angle_max", 360)
        )
        duration = random.uniform(
            config.get("move_duration_min", 0),
            config.get("move_duration_max", 10)
        )
        
        return (round(angle, 1), round(duration, 1))


if __name__ == "__main__":
    # 测试代码（需要机器狗运行）
    print("\n=== Dog Commander Test ===\n")
    
    # 使用模拟模式测试
    from dog.motion_adapter import MotionAdapter
    
    adapter = MotionAdapter()
    
    # 测试移动
    result = adapter.move(angle=45, duration=2)
    print(f"Move result: {result}")
    
    # 测试停止
    result = adapter.stop()
    print(f"Stop result: {result}")
    
    print("\nTest completed (mock mode)")
