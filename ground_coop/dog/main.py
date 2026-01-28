#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Machine Dog Main Program

机器狗端主程序，监听无人机命令并控制运动。
"""

import sys
import time
import signal
import threading
from pathlib import Path

# 确保common模块可导入
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from common import (
    get_logger, get_config, parse_message,
    msg_status, msg_move_done, msg_heartbeat_ack,
    TCPClient, TCPServer, TCPConnection,
    HeartbeatManager, HeartbeatCallback
)
from dog import MotionAdapter


class DogServer:
    """机器狗服务端"""
    
    def __init__(self, config):
        """
        初始化
        
        Args:
            config: 配置管理器
        """
        self.config = config
        self.logger = get_logger("dog")
        
        # 获取配置
        self.port = config.get_port("to_dog", "dog")
        self.heartbeat_config = config.get_heartbeat_config()
        
        # 组件
        self.motion = MotionAdapter()
        self.server: TCPServer = None
        self.drone_client: Optional[TCPClient] = None
        
        # 状态
        self.running = False
        self.connected_drone: Optional[TCPConnection] = None
        
        # 心跳
        self.heartbeat = HeartbeatManager(
            interval=self.heartbeat_config.get("interval", 10),
            timeout=self.heartbeat_config.get("timeout", 30),
            callback=HeartbeatCallback(
                on_timeout=self._on_heartbeat_timeout
            )
        )
        
        # 回传模式: True=中转模式(通过无人机转发), False=直接回传PC
        self.relay_mode = True
        
        self.logger.info("Dog server initialized")
    
    def _on_heartbeat_timeout(self):
        """心跳超时处理"""
        self.logger.warning("Heartbeat timeout from drone")
        # 超时后停止移动
        self.motion.stop()
    
    def start(self):
        """启动服务端"""
        self.logger.info(f"Starting dog server on port {self.port}...")
        
        self.server = TCPServer(port=self.port)
        self.server.set_callbacks(
            on_client_connected=self._on_drone_connected,
            on_client_disconnected=self._on_drone_disconnected,
            on_data=self._on_data_received
        )
        
        if not self.server.start():
            self.logger.error("Failed to start server")
            return False
        
        self.running = True
        self.heartbeat.start()
        
        self.logger.info(f"Dog server started on port {self.port}")
        return True
    
    def stop(self):
        """停止服务端"""
        self.logger.info("Stopping dog server...")
        self.running = False
        self.heartbeat.stop()
        
        if self.server:
            self.server.stop()
        
        if self.drone_client:
            self.drone_client.close()
        
        self.logger.info("Dog server stopped")
    
    def _on_drone_connected(self, conn: TCPConnection):
        """无人机连接回调"""
        self.logger.info(f"Drone connected: {conn.address}")
        self.connected_drone = conn
        self.heartbeat.mark_received()
    
    def _on_drone_disconnected(self, conn: TCPConnection):
        """无人机断开回调"""
        self.logger.info(f"Drone disconnected: {conn.address}")
        self.connected_drone = None
        self.heartbeat.reset()
    
    def _on_data_received(self, conn: TCPConnection, data: bytes):
        """数据接收回调"""
        try:
            text = data.decode('utf-8')
            msg = parse_message(text)
            
            if msg is None:
                self.logger.warning(f"Invalid message: {text}")
                return
            
            self.heartbeat.mark_received()
            
            self.logger.info(f"Received message: {msg.type}")
            
            # 处理消息
            self._handle_message(msg)
            
        except Exception as e:
            self.logger.error(f"Error handling message: {e}")
    
    def _handle_message(self, msg):
        """处理消息"""
        msg_type = msg.type
        
        if msg_type == "move":
            self._handle_move(msg)
        elif msg_type == "return":
            self._handle_return(msg)
        elif msg_type == "stop":
            self._handle_stop(msg)
        elif msg_type == "heartbeat":
            self._handle_heartbeat(msg)
        else:
            self.logger.warning(f"Unknown message type: {msg_type}")
    
    def _handle_move(self, msg):
        """处理移动命令"""
        angle = msg.payload.get("angle", 0)
        duration = msg.payload.get("duration", 5)
        
        self.logger.info(f"Moving: angle={angle}°, duration={duration}s")
        
        result = self.motion.move(angle, duration)
        
        response = msg_move_done(
            success=result.get("success", False),
            position=result.get("position", {})
        )
        
        self._send_response(response)
    
    def _handle_return(self, msg):
        """处理返航命令"""
        self.logger.info("Returning home...")
        
        result = self.motion.return_home()
        
        response = msg_move_done(
            success=result.get("success", False),
            position=result.get("position", {})
        )
        
        self._send_response(response)
    
    def _handle_stop(self, msg):
        """处理停止命令"""
        self.logger.info("Stopping...")
        result = self.motion.stop()
        
        response = msg_move_done(
            success=result.get("success", False),
            position=self.motion.current_position.__dict__
        )
        
        self._send_response(response)
    
    def _handle_heartbeat(self, msg):
        """处理心跳"""
        self.heartbeat.mark_received()
        self._send_response(msg_heartbeat_ack())
    
    def _send_response(self, response):
        """发送响应"""
        if self.connected_drone:
            try:
                self.connected_drone.send(response.to_json().encode('utf-8'))
            except Exception as e:
                self.logger.error(f"Failed to send response: {e}")
    
    def send_status(self, state: str, extra: dict = None):
        """发送状态"""
        status = msg_status(
            state=state,
            position=self.motion.current_position.__dict__,
            extra=extra
        )
        self._send_response(status)


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Machine Dog Server")
    parser.add_argument("--config", type=str, default=None, help="Config file path")
    parser.add_argument("--port", type=int, default=None, help="Listen port")
    parser.add_argument("--auto", action="store_true", help="Auto detect WiFi")
    parser.add_argument("--wifi", type=str, help="Specify WiFi network")
    parser.add_argument("--log-level", type=str, default="INFO", help="Log level")
    
    args = parser.parse_args()
    
    # 初始化日志
    logger = get_logger("dog", level=args.log_level)
    logger.info("=== Machine Dog Server Starting ===")
    
    # 初始化配置
    config = get_config(args.config)
    
    # 应用参数
    if args.port:
        config.set_override("dog", {"ip": "0.0.0.0", "port": args.port})
    if args.wifi:
        config.detect_wifi()
    
    # 打印配置
    dog_config = config.get_device_config("dog")
    logger.info(f"Dog config: {dog_config}")
    
    # 创建并启动服务端
    server = DogServer(config)
    
    # 信号处理
    def signal_handler(sig, frame):
        logger.info("Received shutdown signal")
        server.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        if server.start():
            logger.info("Dog server is running. Press Ctrl+C to stop.")
            
            # 保持运行
            while server.running:
                time.sleep(1)
    except Exception as e:
        logger.error(f"Server error: {e}")
    finally:
        server.stop()
        logger.info("=== Dog Server Stopped ===")


if __name__ == "__main__":
    main()
