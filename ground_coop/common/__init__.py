"""
Ground-Air Cooperation System - Common Modules

Common utilities including:
- config_manager: Configuration management
- logger: Dual-location logging
- protocol: Message protocol definitions
- heartbeat: Heartbeat detection
- tcp_base: TCP client/server base classes
"""

__version__ = "1.0.0"

from .config_manager import ConfigManager, get_config
from .logger import Logger, get_logger
from .protocol import (
    MessageType,
    Message,
    create_message,
    parse_message,
    validate_message,
    msg_start,
    msg_stop,
    msg_control,
    msg_return,
    msg_move,
    msg_move_done,
    msg_status,
    msg_heartbeat,
    msg_heartbeat_ack,
    msg_error,
    msg_quit,
    DeviceState,
    ErrorCode
)
from .heartbeat import HeartbeatManager, HeartbeatCallback
from .tcp_base import TCPClient, TCPServer, TCPConnection

__all__ = [
    'ConfigManager',
    'get_config',
    'Logger',
    'get_logger',
    'MessageType',
    'Message',
    'create_message',
    'parse_message',
    'validate_message',
    'msg_start',
    'msg_stop',
    'msg_control',
    'msg_return',
    'msg_move',
    'msg_move_done',
    'msg_status',
    'msg_heartbeat',
    'msg_heartbeat_ack',
    'msg_error',
    'msg_quit',
    'DeviceState',
    'ErrorCode',
    'HeartbeatManager',
    'HeartbeatCallback',
    'TCPClient',
    'TCPServer',
    'TCPConnection',
]
