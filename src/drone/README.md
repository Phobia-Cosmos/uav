# Drone Module (无人机端)

无人机端主程序，运行在香橙派上，负责飞控控制和机器狗协调。

## 文件说明

| 文件 | 功能描述 |
|-----|---------|
| `drone_server.py` | **核心程序**。TCP服务端，接收PC端指令，控制飞控和机器狗。支持起飞、降落、返航、转向、怠速测试等功能。包含状态广播和传感器更新线程。 |
| `fly.py` | 飞行控制主程序。集成摄像头（OpenCV红色目标识别）和超声波测距，实现基础的自动飞行和避障功能。 |
| `yaw_controller.py` | 航向控制扩展模块。提供三种转向模式：持续旋转、指定角度旋转、完整旋转机动（起飞→悬停→旋转→降落）。 |
| `noGPS.py` | 无GPS环境飞行脚本。禁用RC和失控保护，使用STABILIZE模式在室内环境解锁和测试电机。 |

## 核心类

### DroneServer

```python
class DroneServer:
    def __init__(self, host='0.0.0.0', port=5000)
    def connect_vehicle(self, connection_string, baud=921600)
    def start_server(self)
    def arm_and_takeoff(self, altitude)
    def land(self)
    def return_to_launch(self)
    def set_rc_override(self, roll, pitch, throttle, yaw)
    def emergency_stop(self)
```

### YawController

```python
class YawController:
    def __init__(self, vehicle)
    def rotate_continuous(self, direction, speed, duration)
    def rotate_angle(self, angle, direction, speed, relative)
    def execute_rotation_maneuver(self, altitude, angle, direction, speed)
```

## 运行方式

```bash
# 香橙派端启动
python3 src/drone/drone_server.py --connection /dev/ttyACM0

# 摄像头+超声波飞行测试
python3 src/drone/fly.py

# 无GPS解锁测试
python3 src/drone/noGPS.py
```

## 依赖

- `dronekit>=2.9.0`
- `pymavlink>=2.4.0`
- `pyserial>=3.5`
- `opencv-python` (仅fly.py)
- `RPi.GPIO` (仅fly.py)
