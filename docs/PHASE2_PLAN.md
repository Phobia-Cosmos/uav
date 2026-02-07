# 阶段二：TCP 通信协同

## 1. 概述

本阶段目标：
- 实现机器狗与无人机之间的 TCP 通信
- 机器狗主动发送路径给无人机
- 无人机持续监听来自机器狗的路径数据
- PC 端可同时查看两条路径

## 2. 通信架构

```
┌─────────────────────────────────────────────────────────┐
│                     网络架构                              │
├─────────────────────────────────────────────────────────┤
│                                                         │
│    192.168.55.0/24 网段                                │
│                                                         │
│   ┌─────────────┐         ┌─────────────┐             │
│   │   机器狗     │  TCP   │   无人机     │             │
│   │  192.168.55.127 │ ──────→ │ 192.168.55.128 │             │
│   │  (客户端)     │ 5300   │  (服务端)    │             │
│   └─────────────┘         └─────────────┘             │
│                                         │              │
│                                         ↓              │
│                                 ┌─────────────┐       │
│                                 │     PC      │       │
│                                 │ 192.168.55.126 │       │
│                                 │  (查看)      │       │
│                                 └─────────────┘       │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

## 3. 消息格式定义

### 3.1 路径消息 (PATH)

机器狗发送给无人机的路径数据：

```json
{
  "type": "path",
  "sender": "dog",
  "timestamp": 1700000000,
  "data": {
    "from": {"x": 0, "y": 0},
    "to": {"x": 40, "y": 40},
    "dimension": "2d",
    "path": [
      {"x": 0, "y": 0},
      {"x": 1, "y": 0},
      {"x": 2, "y": 1},
      {"x": 3, "y": 1},
      ...
    ],
    "meta": {
      "total_length": 48.5,
      "point_count": 45,
      "algorithm": "A*",
      "score": {
        "total": 0.823,
        "length": 0.891,
        "smoothness": 0.772,
        "energy": 0.650
      }
    }
  }
}
```

### 3.2 确认消息 (ACK)

无人机确认收到路径：

```json
{
  "type": "ack",
  "receiver": "dog",
  "original_message_id": "msg_001",
  "status": "received",
  "timestamp": 1700000001
}
```

### 3.3 心跳消息 (HEARTBEAT)

连接保活检测：

```json
{
  "type": "heartbeat",
  "sender": "dog",
  "timestamp": 1700000000,
  "payload": {
    "device": "dog",
    "status": "ready"
  }
}
```

### 3.4 请求消息 (REQUEST)

请求对方路径数据：

```json
{
  "type": "request",
  "sender": "uav",
  "timestamp": 1700000000,
  "data": {
    "request_type": "path",
    "from": {"x": 0, "y": 0, "z": 0},
    "to": {"x": 40, "y": 40, "z": 10}
  }
}
```

## 4. 代码结构

```
simulation_tcp/
├── tcp_base.py           # TCP 客户端/服务端基类
├── dog_client.py         # 机器狗客户端
├── uav_server.py         # 无人机服务端
├── path_message.py      # 消息封装/解析
├── shared_config.py      # 共享配置
├── test_communication.py # 通信测试
└── README.md
```

## 5. TCP 基类实现

### 5.1 消息封装

```python
# path_message.py

import json
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional
from enum import Enum
import time
import uuid


class MessageType(Enum):
    PATH = "path"
    ACK = "ack"
    HEARTBEAT = "heartbeat"
    REQUEST = "request"
    ERROR = "error"


@dataclass
class Point2D:
    x: float
    y: float


@dataclass
class Point3D:
    x: float
    y: float
    z: float


@dataclass
class PathMeta:
    total_length: float
    point_count: int
    algorithm: str
    score: Dict[str, float]


@dataclass
class PathMessage:
    """路径消息"""
    type: str
    sender: str
    timestamp: float
    data: Dict

    @classmethod
    def create_path(cls, sender: str, from_pos: Dict, to_pos: Dict,
                    path_points: List[Dict], meta: Dict) -> 'PathMessage':
        return cls(
            type=MessageType.PATH.value,
            sender=sender,
            timestamp=time.time(),
            data={
                "from": from_pos,
                "to": to_pos,
                "dimension": "2d" if "z" not in path_points[0] else "3d",
                "path": path_points,
                "meta": meta
            }
        )

    @classmethod
    def create_ack(cls, receiver: str, original_id: str, status: str) -> 'PathMessage':
        return cls(
            type=MessageType.ACK.value,
            sender="system",
            timestamp=time.time(),
            data={
                "receiver": receiver,
                "original_message_id": original_id,
                "status": status
            }
        )

    @classmethod
    def create_heartbeat(cls, sender: str, device_status: str = "ready") -> 'PathMessage':
        return cls(
            type=MessageType.HEARTBEAT.value,
            sender=sender,
            timestamp=time.time(),
            data={
                "device": sender,
                "status": device_status
            }
        )

    def to_json(self) -> str:
        return json.dumps(self, default=str)

    @classmethod
    def from_json(cls, json_str: str) -> 'PathMessage':
        data = json.loads(json_str)
        return cls(**data)
```

### 5.2 TCP 客户端

```python
# tcp_base.py

import socket
import threading
import json
from typing import Callable, Optional


class TCPClient:
    """TCP 客户端基类"""

    def __init__(self, host: str, port: int,
                 on_message: Callable[[str], None] = None):
        self.host = host
        self.port = port
        self.on_message = on_message
        self.socket = None
        self.connected = False
        self.running = False
        self.receive_thread = None

    def connect(self) -> bool:
        """连接到服务器"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10)
            self.socket.connect((self.host, self.port))
            self.connected = True
            self.running = True
            self._start_receive()
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        self.running = False
        self.connected = False
        if self.socket:
            self.socket.close()

    def send(self, message: str) -> bool:
        """发送消息"""
        if not self.connected:
            return False
        try:
            self.socket.sendall(message.encode('utf-8'))
            return True
        except Exception as e:
            print(f"发送失败: {e}")
            return False

    def _start_receive(self):
        """启动接收线程"""
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()

    def _receive_loop(self):
        """接收消息循环"""
        while self.running and self.connected:
            try:
                data = self.socket.recv(4096)
                if not data:
                    break
                message = data.decode('utf-8')
                if self.on_message:
                    self.on_message(message)
            except socket.timeout:
                continue
            except Exception:
                break
        self.connected = False
```

### 5.3 TCP 服务端

```python
class TCPServer:
    """TCP 服务端基类"""

    def __init__(self, host: str = "0.0.0.0", port: int = 0,
                 on_client_connect: Callable = None,
                 on_client_disconnect: Callable = None,
                 on_message: Callable[[str], None] = None):
        self.host = host
        self.port = port
        self.on_client_connect = on_client_connect
        self.on_client_disconnect = on_client_disconnect
        self.on_message = on_message
        self.socket = None
        self.running = False
        self.clients = []
        self.lock = threading.Lock()

    def start(self) -> bool:
        """启动服务端"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.listen(5)
            self.socket.settimeout(1)
            self.running = True
            threading.Thread(target=self._accept_loop, daemon=True).start()
            return True
        except Exception as e:
            print(f"启动失败: {e}")
            return False

    def stop(self):
        """停止服务端"""
        self.running = False
        for client in self.clients[:]:
            client.close()
        if self.socket:
            self.socket.close()

    def broadcast(self, message: str, exclude: socket.socket = None):
        """广播消息给所有客户端"""
        with self.lock:
            for client in self.clients:
                if client != exclude:
                    try:
                        client.sendall(message.encode('utf-8'))
                    except:
                        pass

    def _accept_loop(self):
        """接受连接循环"""
        while self.running:
            try:
                client_socket, addr = self.socket.accept()
                with self.lock:
                    self.clients.append(client_socket)
                if self.on_client_connect:
                    self.on_client_connect(addr)
                threading.Thread(
                    target=self._handle_client,
                    args=(client_socket,),
                    daemon=True
                ).start()
            except socket.timeout:
                continue
            except Exception:
                break

    def _handle_client(self, client_socket: socket.socket):
        """处理客户端消息"""
        while self.running:
            try:
                data = client_socket.recv(4096)
                if not data:
                    break
                message = data.decode('utf-8')
                if self.on_message:
                    self.on_message(message)
            except Exception:
                break
        with self.lock:
            if client_socket in self.clients:
                self.clients.remove(client_socket)
        client_socket.close()
```

## 6. 机器狗客户端实现

```python
# dog_client.py

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from tcp_base import TCPClient
from path_message import PathMessage
from a_star_2d import AStar2D
import json


class DogClient:
    """机器狗客户端"""

    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.client = TCPClient(host, port)
        self.client.on_message = self._on_message

    def _on_message(self, message: str):
        """处理收到的消息"""
        try:
            msg = PathMessage.from_json(message)
            if msg.type == "ack":
                print(f"[Dog] 收到确认: {msg.data['status']}")
            elif msg.type == "request":
                print(f"[Dog] 收到路径请求")
        except Exception as e:
            print(f"[Dog] 消息解析错误: {e}")

    def connect(self) -> bool:
        """连接到无人机"""
        return self.client.connect()

    def send_path(self, path_data: dict, meta: dict):
        """发送路径给无人机"""
        from_pos = path_data["points"][0]
        to_pos = path_data["points"][-1]

        msg = PathMessage.create_path(
            sender="dog",
            from_pos=from_pos,
            to_pos=to_pos,
            path_points=path_data["points"],
            meta=meta
        )
        self.client.send(msg.to_json())
        print(f"[Dog] 路径已发送: {len(path_data['points'])} 个点")

    def disconnect(self):
        """断开连接"""
        self.client.disconnect()


def main():
    """主函数"""
    import map_generator

    # 加载地图
    config_path = os.path.join(os.path.dirname(__file__), "config/scenario_01.json")
    map_data = map_generator.load(config_path)

    # 规划路径
    planner = AStar2D(map_data["dog_obstacles"], (50, 50))
    path = planner.plan(
        (map_data["start"]["x"], map_data["start"]["y"]),
        (map_data["goal"]["x"], map_data["goal"]["y"])
    )

    if path:
        # 评估路径
        from path_evaluator import PathEvaluator
        evaluator = PathEvaluator()
        score = evaluator.evaluate_2d(path)

        # 发送到无人机
        client = DogClient("192.168.55.128", 5300)
        if client.connect():
            client.send_path(path, score)
            client.disconnect()


if __name__ == "__main__":
    main()
```

## 7. 无人机服务端实现

```python
# uav_server.py

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from tcp_base import TCPServer
from path_message import PathMessage
from visualizer import PathVisualizer
import matplotlib.pyplot as plt
import json


class UAVServer:
    """无人机服务端"""

    def __init__(self, port: int = 5300):
        self.port = port
        self.server = TCPServer(port=port)
        self.server.on_message = self._on_message
        self.server.on_client_connect = self._on_connect
        self.dog_path = None
        self.uav_path = None

    def _on_connect(self, addr):
        print(f"[UAV] 机器狗已连接: {addr}")

    def _on_message(self, message: str):
        """处理收到的消息"""
        try:
            msg = PathMessage.from_json(message)
            if msg.type == "path" and msg.sender == "dog":
                print(f"[UAV] 收到机器狗路径: {len(msg.data['path'])} 个点")
                self.dog_path = {
                    "points": msg.data["path"],
                    "meta": msg.data["meta"],
                    "color": "orange"
                }
                # 发送确认
                ack = PathMessage.create_ack("dog", "path", "received")
                self.server.broadcast(ack.to_json())
        except Exception as e:
            print(f"[UAV] 消息解析错误: {e}")

    def start(self):
        """启动服务端"""
        print(f"[UAV] 启动服务端，监听端口 {self.port}...")
        self.server.start()

    def stop(self):
        """停止服务端"""
        self.server.stop()

    def visualize(self):
        """可视化路径"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

        # 2D 机器狗路径
        if self.dog_path:
            points = self.dog_path["points"]
            xs = [p["x"] for p in points]
            ys = [p["y"] for p in points]
            ax1.plot(xs, ys, 'orange', linewidth=2, label='Dog Path')
            ax1.scatter(xs, ys, c='orange', s=30)

        ax1.set_title("Dog 2D Path")
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')

        # 3D 无人机路径 (这里只显示2D作为示例)
        ax2.set_title("UAV 3D Path (TBD)")
        ax2.legend()

        plt.tight_layout()
        plt.show()


def main():
    """主函数"""
    server = UAVServer(5300)
    server.start()

    print("[UAV] 按 Ctrl+C 停止")
    try:
        while True:
            plt.pause(1)
    except KeyboardInterrupt:
        server.stop()


if __name__ == "__main__":
    main()
```

## 8. 通信测试

```python
# test_communication.py

import sys
import os
import threading
import time
sys.path.insert(0, os.path.dirname(__file__))

from dog_client import DogClient
from uav_server import UAVServer


def test_communication():
    """测试通信"""

    # 启动服务端
    server = UAVServer(5300)
    server.start()
    print("[Test] 服务端已启动")

    time.sleep(1)

    # 启动客户端
    client = DogClient("127.0.0.1", 5300)
    if client.connect():
        print("[Test] 客户端已连接")

        # 发送测试路径
        test_path = {
            "points": [
                {"x": 0, "y": 0},
                {"x": 5, "y": 5},
                {"x": 10, "y": 10}
            ]
        }
        test_meta = {
            "total_length": 14.14,
            "point_count": 3,
            "algorithm": "A*",
            "score": {"total": 0.8}
        }

        client.send_path(test_path, test_meta)
        time.sleep(1)

        client.disconnect()
        print("[Test] 测试完成")

    server.stop()


if __name__ == "__main__":
    test_communication()
```

## 9. 运行说明

### 9.1 启动无人机服务端

```bash
cd simulation_tcp
python uav_server.py
```

输出：
```
[UAV] 启动服务端，监听端口 5300...
[UAV] 按 Ctrl+C 停止
```

### 9.2 启动机器狗客户端

```bash
cd simulation_tcp
python dog_client.py
```

输出：
```
[Dog] 连接到 192.168.55.128:5300
[Dog] 路径已发送: 45 个点
```

### 9.3 联合测试

```bash
# 终端 1 - 启动无人机
python uav_server.py

# 终端 2 - 启动机器狗
python dog_client.py
```

## 10. 与阶段一的差异

| 项目 | 阶段一 | 阶段二 |
|------|--------|--------|
| 数据流向 | 单机生成 | 机器狗 → 无人机 |
| 通信方式 | 无 | TCP |
| 路径展示 | 分开 | 统一界面显示 |
| 决策方式 | 各自展示 | 机器狗主动发送 |

## 11. 后续扩展

1. **PC 端集成**
   - PC 端也可以作为客户端接收路径
   - 实现三端统一的可视化界面

2. **消息可靠性**
   - 添加消息重传机制
   - 实现消息序号和去重

3. **错误处理**
   - 连接断开自动重连
   - 超时处理机制
