# Common Modules (公共模块)

地空协同系统的公共模块，提供配置管理、日志、协议、网络通信等基础功能。

## 文件说明

| 文件 | 功能描述 |
|-----|---------|
| `__init__.py` | 模块入口，导出所有公共类和函数 |
| `protocol.py` | JSON消息协议定义，包括消息类型枚举、消息封装/解析、便捷函数 |
| `tcp_base.py` | TCP客户端和服务端基类封装，支持连接管理、心跳重连 |
| `heartbeat.py` | 心跳检测模块，用于设备在线状态监控和超时处理 |
| `config_manager.py` | 配置管理，支持多WiFi配置文件智能选择 |
| `logger.py` | 双位置日志，同时输出到文件和journalctl |

## 使用示例

```python
from common import (
    get_logger,
    get_config,
    TCPClient,
    TCPServer,
    HeartbeatManager,
    Message,
    msg_start,
    msg_status,
    parse_message
)
```

## Protocol 模块

### 消息类型 (MessageType)

| 类型 | 说明 |
|-----|------|
| `START` | 起飞指令 |
| `STOP` | 停止指令 |
| `CONTROL` | 控制模式 |
| `RETURN` | 返航指令 |
| `EMERGENCY_STOP` | 紧急停止 |
| `STATUS` | 状态回传 |
| `HEARTBEAT` | 心跳 |
| `ERROR` | 错误信息 |
| `MOVE_DONE` | 移动完成 |

### 便捷函数

```python
# 创建消息
msg = msg_start(altitude=3.0)
msg = msg_status(state="hovering", battery=85)
msg = msg_error("CONNECTION_LOST", "PC disconnected")

# 解析消息
msg = parse_message('{"msg_type": "start", ...}')

# JSON转换
json_str = msg.to_json()
```

## TCP Base 模块

### TCPServer

```python
server = TCPServer("0.0.0.0", 5000)
server.set_callbacks(
    on_client_connected=callback,
    on_data=data_handler
)
server.start()
```

### TCPClient

```python
client = TCPClient("192.168.1.100", 5000)
client.set_callbacks(
    on_connected=cb,
    on_disconnected=cb,
    on_data=data_handler
)
client.connect()
```

## Heartbeat 模块

```python
heartbeat = HeartbeatManager(
    interval=10,      # 发送间隔（秒）
    timeout=30,       # 超时时间（秒）
    callback=HeartbeatCallback(
        on_send=send_heartbeat,
        on_timeout=handle_timeout
    )
)
heartbeat.start()
```

## Config Manager 模块

```python
config = ConfigManager("config/network_config.json")
pc_ip = config.get_ip("pc")
drone_port = config.get_port("from_drone", "pc")
```
