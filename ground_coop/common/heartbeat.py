#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Heartbeat Module

心跳检测模块，用于检测连接存活状态。
"""

import time
import threading
from typing import Callable, Optional, Dict
from dataclasses import dataclass, field
from enum import Enum


class HeartbeatState(Enum):
    """心跳状态"""
    STOPPED = "stopped"
    RUNNING = "running"
    TIMEOUT = "timeout"
    RECEIVED = "received"


@dataclass
class HeartbeatCallback:
    """心跳回调函数"""
    on_timeout: Optional[Callable[[], None]] = None
    on_received: Optional[Callable[[], None]] = None
    on_send: Optional[Callable[[], None]] = None


class HeartbeatManager:
    """心跳管理器"""
    
    def __init__(
        self,
        interval: float = 10.0,
        timeout: float = 30.0,
        callback: HeartbeatCallback = None
    ):
        """
        初始化心跳管理器
        
        Args:
            interval: 心跳发送间隔（秒）
            timeout: 超时时间（秒）
            callback: 回调函数
        """
        self.interval = interval
        self.timeout = timeout
        self.callback = callback or HeartbeatCallback()
        
        self._last_received_time: float = 0
        self._last_sent_time: float = 0
        self._state = HeartbeatState.STOPPED
        self._lock = threading.Lock()
        self._timer: Optional[threading.Timer] = None
        self._running = False
        
        self._timeout_handled = False

    @property
    def state(self) -> HeartbeatState:
        """获取当前状态"""
        with self._lock:
            if not self._running:
                return HeartbeatState.STOPPED
            
            if self.is_timeout():
                return HeartbeatState.TIMEOUT
            
            return HeartbeatState.RUNNING

    @property
    def last_received_time(self) -> float:
        """获取最后收到心跳的时间"""
        return self._last_received_time

    @property
    def last_sent_time(self) -> float:
        """获取最后发送心跳的时间"""
        return self._last_sent_time

    def is_timeout(self) -> bool:
        """检查是否超时"""
        if self._last_received_time == 0:
            return False
        return (time.time() - self._last_received_time) > self.timeout

    def is_alive(self) -> bool:
        """检查连接是否存活"""
        return self._running and not self.is_timeout()

    def mark_received(self):
        """标记收到心跳"""
        with self._lock:
            self._last_received_time = time.time()
            self._timeout_handled = False
            
            if self.callback.on_received:
                self.callback.on_received()

    def mark_sent(self):
        """标记发送心跳"""
        self._last_sent_time = time.time()
        
        if self.callback.on_send:
            self.callback.on_send()

    def _send_heartbeat(self):
        """发送心跳（内部调用）"""
        if not self._running:
            return
        
        self.mark_sent()
        
        if self.callback.on_send:
            self.callback.on_send()
        
        # 调度下一次发送
        self._schedule_next()

    def _schedule_next(self):
        """调度下一次心跳发送"""
        if not self._running:
            return
        
        self._timer = threading.Timer(
            self.interval,
            self._send_heartbeat
        )
        self._timer.daemon = True
        self._timer.start()

    def start(self):
        """启动心跳检测"""
        with self._lock:
            if self._running:
                return
            
            self._running = True
            self._last_received_time = time.time()
            self._timeout_handled = False
            self._state = HeartbeatState.RUNNING
            
            self._schedule_next()

    def stop(self):
        """停止心跳检测"""
        with self._lock:
            self._running = False
            self._state = HeartbeatState.STOPPED
            
            if self._timer:
                self._timer.cancel()
                self._timer = None

    def reset(self):
        """重置心跳状态"""
        with self._lock:
            self._last_received_time = 0
            self._last_sent_time = 0
            self._timeout_handled = False

    def check_and_handle_timeout(self) -> bool:
        """
        检查并处理超时
        
        Returns:
            是否发生超时
        """
        if self.is_timeout() and not self._timeout_handled:
            self._timeout_handled = True
            
            if self.callback.on_timeout:
                self.callback.on_timeout()
            
            return True
        
        return False

    def get_remaining_time(self) -> float:
        """获取到下一次心跳的超时时间"""
        if not self._running:
            return 0
        
        if self._last_received_time == 0:
            return self.timeout
        
        remaining = self.timeout - (time.time() - self._last_received_time)
        return max(0, remaining)


class MultiHeartbeatManager:
    """多设备心跳管理器"""

    def __init__(
        self,
        interval: float = 10.0,
        timeout: float = 30.0
    ):
        """
        初始化多设备心跳管理器
        
        Args:
            interval: 心跳间隔
            timeout: 超时时间
        """
        self.interval = interval
        self.timeout = timeout
        self._managers: Dict[str, HeartbeatManager] = {}
        self._lock = threading.Lock()

    def add_device(
        self,
        device_id: str,
        on_timeout: Callable[[], None] = None
    ) -> HeartbeatManager:
        """
        添加设备心跳管理
        
        Args:
            device_id: 设备ID
            on_timeout: 超时回调
            
        Returns:
            HeartbeatManager实例
        """
        callback = HeartbeatCallback(on_timeout=on_timeout)
        
        with self._lock:
            if device_id in self._managers:
                self._managers[device_id].stop()
            
            self._managers[device_id] = HeartbeatManager(
                interval=self.interval,
                timeout=self.timeout,
                callback=callback
            )
            
            return self._managers[device_id]

    def remove_device(self, device_id: str):
        """移除设备"""
        with self._lock:
            if device_id in self._managers:
                self._managers[device_id].stop()
                del self._managers[device_id]

    def get_manager(self, device_id: str) -> Optional[HeartbeatManager]:
        """获取设备心跳管理器"""
        return self._managers.get(device_id)

    def mark_received(self, device_id: str):
        """标记设备收到心跳"""
        manager = self.get_manager(device_id)
        if manager:
            manager.mark_received()

    def start_all(self):
        """启动所有设备心跳"""
        with self._lock:
            for manager in self._managers.values():
                manager.start()

    def stop_all(self):
        """停止所有设备心跳"""
        with self._lock:
            for manager in self._managers.values():
                manager.stop()

    def check_timeouts(self) -> list:
        """
        检查所有设备超时情况
        
        Returns:
            超时设备列表
        """
        timed_out = []
        
        with self._lock:
            for device_id, manager in self._managers.items():
                if manager.check_and_handle_timeout():
                    timed_out.append(device_id)
        
        return timed_out


if __name__ == "__main__":
    # 测试代码
    print("=== Heartbeat Module Test ===\n")
    
    timeout_devices = []
    
    def on_timeout(device_id):
        print(f"Device {device_id} timeout!")
        timeout_devices.append(device_id)
    
    # 创建管理器
    mgr = MultiHeartbeatManager(interval=1.0, timeout=3.0)
    
    # 添加设备
    mgr.add_device("drone", lambda: on_timeout("drone"))
    mgr.add_device("dog", lambda: on_timeout("dog"))
    
    # 启动
    mgr.start_all()
    
    print("Heartbeat managers started...")
    
    # 模拟收到心跳
    time.sleep(0.5)
    mgr.mark_received("drone")
    print("Drone heartbeat received")
    
    # 等待超时
    time.sleep(4)
    
    print(f"\nTimed out devices: {timeout_devices}")
    print(f"Drone alive: {mgr.get_manager('drone').is_alive()}")
    print(f"Dog alive: {mgr.get_manager('dog').is_alive()}")
    
    # 停止
    mgr.stop_all()
    print("\nHeartbeat managers stopped")
