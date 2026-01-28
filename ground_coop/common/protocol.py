#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Message Protocol Definitions

定义地空协同系统的JSON消息协议。
"""

import json
import uuid
import time
from enum import Enum
from dataclasses import dataclass, field, asdict
from typing import Optional, Dict, Any, List


class MessageType(str, Enum):
    """消息类型枚举"""
    
    # 命令类
    START = "start"
    STOP = "stop"
    CONTROL = "control"
    RETURN = "return"
    EMERGENCY_STOP = "emergency_stop"
    TEST = "test"
    STOP_TEST = "stop_test"
    STATUS_REQUEST = "status_request"
    RESET = "reset"
    
    # 状态类
    STATUS = "status"
    MOVE_DONE = "move_done"
    HEARTBEAT = "heartbeat"
    HEARTBEAT_ACK = "heartbeat_ack"
    ERROR = "error"
    
    # 控制类
    QUIT = "quit"


@dataclass
class Message:
    """消息数据类"""
    msg_type: str
    timestamp: float = field(default_factory=time.time)
    message_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    payload: Dict[str, Any] = field(default_factory=dict)
    
    def to_json(self) -> str:
        """转换为JSON字符串"""
        return json.dumps(asdict(self), ensure_ascii=False)
    
    @classmethod
    def from_json(cls, json_str: str) -> "Message":
        """从JSON字符串创建消息"""
        data = json.loads(json_str)
        return cls(
            msg_type=data.get("msg_type", ""),
            timestamp=data.get("timestamp", time.time()),
            message_id=data.get("message_id", ""),
            payload=data.get("payload", {})
        )
    
    @property
    def type(self) -> str:
        """获取消息类型"""
        return self.msg_type


def create_message(
    msg_type: str,
    payload: Optional[Dict] = None
) -> Message:
    """
    创建消息
    
    Args:
        msg_type: 消息类型
        payload: 负载数据
        
    Returns:
        Message对象
    """
    return Message(
        msg_type=msg_type,
        payload=payload or {}
    )


def parse_message(json_str: str) -> Optional[Message]:
    """
    解析JSON消息
    
    Args:
        json_str: JSON字符串
        
    Returns:
        Message对象，解析失败返回None
    """
    try:
        return Message.from_json(json_str)
    except (json.JSONDecodeError, KeyError):
        return None


def validate_message(json_str: str) -> tuple:
    """
    验证消息格式
    
    Args:
        json_str: JSON字符串
        
    Returns:
        (是否有效, Message对象或错误信息)
    """
    msg = parse_message(json_str)
    if msg is None:
        return False, "Invalid JSON format"
    
    if not msg.msg_type:
        return False, "Missing message type"
    
    return True, msg


# 便捷函数 - 创建各种消息

def msg_start(altitude: float = 3.0, mode: str = "GUIDED") -> Message:
    """启动消息"""
    return create_message(MessageType.START.value, {"altitude": altitude, "mode": mode})

def msg_stop() -> Message:
    """停止消息"""
    return create_message(MessageType.STOP.value)

def msg_control(mode: str = "auto") -> Message:
    """控制消息"""
    return create_message(MessageType.CONTROL.value, {"mode": mode})

def msg_return() -> Message:
    """返航消息"""
    return create_message(MessageType.RETURN.value)

def msg_move(angle: float, duration: float) -> Message:
    """移动消息"""
    return create_message("move", {
        "angle": angle,
        "duration": duration
    })

def msg_move_done(success: bool, position: Dict = None) -> Message:
    """移动完成消息"""
    return create_message(MessageType.MOVE_DONE.value, {
        "success": success,
        "position": position or {}
    })

def msg_test(filename: str) -> Message:
    """测试消息 - 在无人机端运行指定测试脚本"""
    return create_message(MessageType.TEST.value, {"filename": filename})

def msg_stop_test() -> Message:
    """停止测试消息"""
    return create_message(MessageType.STOP_TEST.value)

def msg_status(
    state: str,
    altitude: Optional[float] = None,
    battery: Optional[int] = None,
    position: Dict = None,
    extra: Dict = None
) -> Message:
    """状态消息"""
    payload = {"state": state}
    if altitude is not None:
        payload["altitude"] = altitude
    if battery is not None:
        payload["battery"] = battery
    if position is not None:
        payload["position"] = position
    if extra is not None:
        payload.update(extra)
    return create_message(MessageType.STATUS.value, payload)

def msg_heartbeat() -> Message:
    """心跳消息"""
    return create_message(MessageType.HEARTBEAT.value)

def msg_heartbeat_ack() -> Message:
    """心跳响应消息"""
    return create_message(MessageType.HEARTBEAT_ACK.value)

def msg_error(error_code: str, error_msg: str) -> Message:
    """错误消息"""
    return create_message(MessageType.ERROR.value, {
        "code": error_code,
        "message": error_msg
    })

def msg_quit() -> Message:
    """退出消息"""
    return create_message(MessageType.QUIT.value)

def msg_status_request() -> Message:
    """状态请求消息"""
    return create_message(MessageType.STATUS_REQUEST.value)

def msg_reset() -> Message:
    """重置消息 - 重置飞控连接和状态"""
    return create_message(MessageType.RESET.value)


# 状态常量
class DeviceState:
    """设备状态常量"""
    
    # 无人机状态
    DRONE_IDLE = "idle"
    DRONE_TAKING_OFF = "taking_off"
    DRONE_HOVERING = "hovering"
    DRONE_MOVING = "moving"
    DRONE_RETURNING = "returning"
    DRONE_LANDING = "landing"
    DRONE_LANDED = "landed"
    DRONE_ARMED = "armed"
    DRONE_DISARMED = "disarmed"
    DRONE_ERROR = "error"
    
    # 机器狗状态
    DOG_IDLE = "idle"
    DOG_MOVING = "moving"
    DOG_RETURNING = "returning"
    DOG_STOPPED = "stopped"
    DOG_ERROR = "error"


# 错误码常量
class ErrorCode:
    """错误码常量"""
    
    CONNECTION_LOST = "CONNECTION_LOST"
    TIMEOUT = "TIMEOUT"
    INVALID_COMMAND = "INVALID_COMMAND"
    DEVICE_NOT_READY = "DEVICE_NOT_READY"
    FLIGHT_ERROR = "FLIGHT_ERROR"
    MOVE_ERROR = "MOVE_ERROR"
    CONFIG_ERROR = "CONFIG_ERROR"
    UNKNOWN = "UNKNOWN"


if __name__ == "__main__":
    # 测试代码
    print("=== Message Protocol Test ===\n")
    
    # 创建消息
    start_msg = msg_start(altitude=5.0)
    print(f"Start message: {start_msg.to_json()}")
    
    status_msg = msg_status(
        state=DeviceState.DRONE_HOVERING,
        altitude=5.0,
        battery=85
    )
    print(f"Status message: {status_msg.to_json()}")
    
    move_msg = msg_move(angle=45, duration=5)
    print(f"Move message: {move_msg.to_json()}")
    
    # 解析消息
    parsed = parse_message('{"msg_type": "start", "payload": {"altitude": 3.0}}')
    if parsed:
        print(f"\nParsed type: {parsed.msg_type}")
        print(f"Parsed payload: {parsed.payload}")
    
    # 验证消息
    valid, result = validate_message('{"msg_type": "heartbeat"}')
    print(f"\nValid heartbeat: {valid}")
    
    valid, result = validate_message("invalid json")
    print(f"Valid invalid: {valid}, error: {result}")
