#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TCP Client/Server Base Classes

TCP客户端和服务端的基类封装。
"""

import socket
import threading
import time
from typing import Optional, Callable, List
from dataclasses import dataclass, field
from enum import Enum


class ConnectionState(Enum):
    """连接状态"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"


@dataclass
class TCPConnection:
    """TCP连接信息"""
    socket: socket.socket
    address: tuple
    connected_at: float = field(default_factory=time.time)
    
    def send(self, data: bytes) -> bool:
        """发送数据"""
        try:
            self.socket.sendall(data)
            return True
        except Exception:
            return False
    
    def recv(self, buffer_size: int = 4096) -> Optional[bytes]:
        """接收数据"""
        try:
            data = self.socket.recv(buffer_size)
            return data if data else None
        except Exception:
            return None
    
    def close(self):
        """关闭连接"""
        try:
            self.socket.close()
        except Exception:
            pass


class TCPClient:
    """TCP客户端基类"""
    
    def __init__(
        self,
        host: str,
        port: int,
        reconnect_interval: float = 5.0,
        max_reconnect_attempts: int = 0
    ):
        """
        初始化TCP客户端
        
        Args:
            host: 服务器地址
            port: 服务器端口
            reconnect_interval: 重连间隔（秒）
            max_reconnect_attempts: 最大重连次数（0表示无限）
        """
        self.host = host
        self.port = port
        self.reconnect_interval = reconnect_interval
        self.max_reconnect_attempts = max_reconnect_attempts
        
        self._socket: Optional[socket.socket] = None
        self._state = ConnectionState.DISCONNECTED
        self._lock = threading.Lock()
        self._receive_thread: Optional[threading.Thread] = None
        self._running = False
        self._reconnect_count = 0
        
        self._on_connected: Optional[Callable[[], None]] = None
        self._on_disconnected: Optional[Callable[[], None]] = None
        self._on_error: Optional[Callable[[str], None]] = None
        self._on_data: Optional[Callable[[bytes], None]] = None

    @property
    def state(self) -> ConnectionState:
        """获取连接状态"""
        return self._state

    @property
    def is_connected(self) -> bool:
        """是否已连接"""
        return self._state == ConnectionState.CONNECTED

    def set_callbacks(
        self,
        on_connected: Callable = None,
        on_disconnected: Callable = None,
        on_error: Callable = None,
        on_data: Callable = None
    ):
        """设置回调函数"""
        self._on_connected = on_connected
        self._on_disconnected = on_disconnected
        self._on_error = on_error
        self._on_data = on_data

    def connect(self) -> bool:
        """连接到服务器"""
        with self._lock:
            if self._state == ConnectionState.CONNECTED:
                return True
            
            self._state = ConnectionState.CONNECTING
            
            try:
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._socket.settimeout(10)
                self._socket.connect((self.host, self.port))
                
                self._state = ConnectionState.CONNECTED
                self._reconnect_count = 0
                
                # 启动接收线程
                self._running = True
                self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
                self._receive_thread.start()
                
                if self._on_connected:
                    self._on_connected()
                
                return True
                
            except Exception as e:
                self._state = ConnectionState.ERROR
                
                if self._on_error:
                    self._on_error(str(e))
                
                return False

    def disconnect(self):
        """断开连接"""
        with self._lock:
            self._running = False
            self._state = ConnectionState.DISCONNECTED
            
            if self._socket:
                try:
                    self._socket.close()
                except Exception:
                    pass
                self._socket = None
            
            if self._on_disconnected:
                self._on_disconnected()

    def _receive_loop(self):
        """接收数据循环"""
        while self._running:
            try:
                data = self._socket.recv(4096)
                if not data:
                    break
                
                if self._on_data:
                    self._on_data(data)
                    
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    pass
                break
        
        if self._running and self._state == ConnectionState.CONNECTED:
            self._handle_disconnect()

    def _handle_disconnect(self):
        """处理断开连接"""
        was_connected = self._state == ConnectionState.CONNECTED
        
        self._state = ConnectionState.DISCONNECTED
        
        if was_connected and self._on_disconnected:
            try:
                self._on_disconnected()
            except:
                pass
        
        # 自动重连
        if self._running and self.max_reconnect_attempts == 0:
            threading.Thread(target=self._reconnect_loop, daemon=True).start()
        elif self._running and self._reconnect_count < self.max_reconnect_attempts:
            self._reconnect_count += 1
            threading.Thread(target=self._reconnect_loop, daemon=True).start()

    def _reconnect_loop(self):
        """重连循环"""
        time.sleep(self.reconnect_interval)
        self.connect()

    def send(self, data: bytes) -> bool:
        """发送数据"""
        if not self.is_connected:
            return False
        
        try:
            self._socket.sendall(data)
            return True
        except Exception:
            self._handle_disconnect()
            return False

    def send_text(self, text: str, encoding: str = "utf-8") -> bool:
        """发送文本"""
        return self.send(text.encode(encoding))

    def close(self):
        """关闭客户端"""
        self.disconnect()


class TCPServer:
    """TCP服务端基类"""
    
    def __init__(self, host: str = "0.0.0.0", port: int = 0):
        """
        初始化TCP服务端
        
        Args:
            host: 监听地址
            port: 监听端口（0表示自动选择端口）
        """
        self.host = host
        self.port = port
        
        self._socket: Optional[socket.socket] = None
        self._state = ConnectionState.DISCONNECTED
        self._lock = threading.Lock()
        self._running = False
        self._connections: List[TCPConnection] = []
        self._accept_thread: Optional[threading.Thread] = None
        
        self._on_client_connected: Optional[Callable[[TCPConnection], None]] = None
        self._on_client_disconnected: Optional[Callable[[TCPConnection], None]] = None
        self._on_error: Optional[Callable[[str], None]] = None
        self._on_data: Optional[Callable[[TCPConnection, bytes], None]] = None

    @property
    def port(self) -> int:
        """获取实际监听端口"""
        return self._port if self._socket else self._port

    @port.setter
    def port(self, value: int):
        self._port = value

    @property
    def actual_port(self) -> int:
        """获取实际绑定的端口"""
        if self._socket:
            return self._socket.getsockname()[1]
        return 0

    @property
    def is_running(self) -> bool:
        """是否正在运行"""
        return self._running

    @property
    def connection_count(self) -> int:
        """连接数量"""
        with self._lock:
            return len(self._connections)

    def set_callbacks(
        self,
        on_client_connected: Callable = None,
        on_client_disconnected: Callable = None,
        on_error: Callable = None,
        on_data: Callable = None
    ):
        """设置回调函数"""
        self._on_client_connected = on_client_connected
        self._on_client_disconnected = on_client_disconnected
        self._on_error = on_error
        self._on_data = on_data

    def start(self) -> bool:
        """启动服务端"""
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.bind((self.host, self._port))
            self._socket.listen(5)
            self._socket.settimeout(1)
            
            self._running = True
            self._state = ConnectionState.CONNECTED
            
            # 启动接受连接线程
            self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
            self._accept_thread.start()
            
            return True
            
        except Exception as e:
            if self._on_error:
                self._on_error(str(e))
            return False

    def stop(self):
        """停止服务端"""
        self._running = False
        
        # 关闭所有连接
        with self._lock:
            for conn in self._connections[:]:
                conn.close()
            self._connections.clear()
        
        # 关闭socket
        if self._socket:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None
        
        self._state = ConnectionState.DISCONNECTED

    def _accept_loop(self):
        """接受连接循环"""
        while self._running:
            try:
                client_socket, addr = self._socket.accept()
                conn = TCPConnection(client_socket, addr)
                
                with self._lock:
                    self._connections.append(conn)
                
                # 启动接收线程
                threading.Thread(
                    target=self._handle_client,
                    args=(conn,),
                    daemon=True
                ).start()
                
                if self._on_client_connected:
                    self._on_client_connected(conn)
                    
            except socket.timeout:
                continue
            except Exception:
                if self._running:
                    continue
                break

    def _handle_client(self, conn: TCPConnection):
        """处理客户端数据"""
        while self._running:
            try:
                data = conn.recv(4096)
                if not data:
                    break
                
                if self._on_data:
                    self._on_data(conn, data)
                    
            except Exception:
                break
        
        # 客户端断开
        with self._lock:
            if conn in self._connections:
                self._connections.remove(conn)
        
        if self._on_client_disconnected:
            self._on_client_disconnected(conn)
        
        conn.close()

    def broadcast(self, data: bytes, exclude: TCPConnection = None):
        """广播数据到所有客户端"""
        with self._lock:
            for conn in self._connections:
                if conn != exclude:
                    conn.send(data)

    def send_to(self, conn: TCPConnection, data: bytes) -> bool:
        """发送数据到指定连接"""
        return conn.send(data)

    def get_connection_by_address(self, address: tuple) -> Optional[TCPConnection]:
        """根据地址获取连接"""
        with self._lock:
            for conn in self._connections:
                if conn.address == address:
                    return conn
        return None

    def close(self):
        """关闭服务端"""
        self.stop()


if __name__ == "__main__":
    # 简单测试
    print("=== TCP Base Test ===\n")
    
    # 创建服务端
    server = TCPServer("127.0.0.1", 0)
    port = server.actual_port
    print(f"Server started on port {port}")
    
    # 设置回调
    def on_client_connected(conn):
        print(f"Client connected: {conn.address}")
    
    def on_data(conn, data):
        print(f"Received from {conn.address}: {data.decode()}")
        # 回显
        conn.send(f"Echo: {data.decode()}".encode())
    
    server.set_callbacks(
        on_client_connected=on_client_connected,
        on_data=on_data
    )
    server.start()
    
    # 创建客户端
    client = TCPClient("127.0.0.1", port)
    
    def on_client_data(data):
        print(f"Client received: {data.decode()}")
    
    client.set_callbacks(on_data=on_client_data)
    
    if client.connect():
        print("Client connected")
        client.send_text("Hello, Server!")
        time.sleep(0.5)
        client.disconnect()
    
    time.sleep(0.5)
    server.stop()
    print("\nTest completed")
