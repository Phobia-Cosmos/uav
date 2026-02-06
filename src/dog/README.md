# Dog Module (机器狗端)

机器狗端控制程序，运行在机器狗上，负责运动控制和ROS通信。

## 文件说明

| 文件 | 功能描述 |
|-----|---------|
| `motion_adapter.py` | **核心程序**。机器狗运动控制适配器，封装ROS服务调用。提供移动、返航、停止等接口，支持模拟模式用于测试。 |

## 核心类

### DogPosition

```python
@dataclass
class DogPosition:
    x: float      # X坐标
    y: float      # Y坐标
    yaw: float    # 航向角
```

### MotionAdapter

```python
class MotionAdapter:
    def __init__(self)
    def move(self, angle, duration)       # 移动指定角度和时长
    def return_home(self)                 # 返航
    def stop(self)                        # 停止
    def get_position(self)                # 获取当前位置
    def get_status(self)                 # 获取状态
```

## 返回值格式

```python
{
    "success": True,
    "position": {
        "x": 1.5,
        "y": 2.3,
        "yaw": 45.0
    }
}
```

## 运行方式

```bash
# 启动机器狗
python3 src/dog/motion_adapter.py
```

## ROS集成

该模块支持两种模式：

1. **ROS模式**（推荐）
   - 自动连接ROS服务
   - 发布速度命令到 `/cmd_vel` topic
   - 订阅里程计信息

2. **模拟模式**（开发测试用）
   - 当ROS不可用时自动切换
   - 模拟返回移动结果
   - 无需真实机器狗即可测试通信

## 通信协议

接收来自无人机的JSON指令：

```json
{
    "msg_type": "move",
    "payload": {
        "angle": 45,      // 移动方向（度）
        "duration": 5     // 移动时长（秒）
    }
}
```
