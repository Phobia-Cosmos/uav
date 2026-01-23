# UAV实时控制系统 - 项目上下文

> 创建日期: 2026-01-23
> 最后更新: 2026-01-23

## 项目概述

实现PC端地面站实时控制无人机（香橙派+飞控）的完整解决方案。

---

## 环境配置

### 当前环境
- **操作位置**: Windows → 即将切换到 Ubuntu 22.04
- **香橙派系统**: OrangePi OS
- **遥控器**: 有（可手动接管）
- **仿真环境**: Ubuntu 22.04 SITL（已安装）
- **开发工具**: dronekit-python, pymavlink

### 网络配置
- **香橙派IP**: 预期从路由器DHCP获取（保持不变）
- **PC IP**: 192.168.1.50（控制端）
- **通信端口**:
  - TCP 5000: PC→香橙派 控制指令
  - TCP 5001: 香橙派→PC 状态回传
  - UDP 14550: MAVLink 飞控通信

---

## 系统架构

```
┌─────────────────────┐         TCP:5000          ┌─────────────────────┐
│    PC (192.168.1.50)│ ────────────────────────> │ 香橙派(动态IP)       │
│  ground_station.py  │          控制指令         │  drone_server.py    │
│    地面站控制       │ <───────────────────────  │    服务端           │
└─────────────────────┘         TCP:5001          └─────────────────────┘
         │                         状态回传                  │
         │                                                  │
         │                UDP:14550                         │
         └──────────────────────────────────────────────────┘
                           MAVLink
                        (飞控通信)
```

---

## 功能需求

### 1. WiFi自动连接
- 香橙派上电后自动连接指定WiFi
- 连接成功后发送状态到PC
- 设置开机自启

### 2. 系统自检
- 飞控连接检查
- 传感器状态检查
- 自检结果回传PC

### 3. 怠速测试
- 不起飞情况下测试电机响应
- 油门范围: 1050-1250 PWM
- 测试时长: 5秒
- 参考 noGPS.py 实现逻辑

### 4. 飞行控制
- 起飞/降落
- RC覆盖控制
- 航向控制（转向）
- 速度控制

### 5. 转向功能
- 顺时针/逆时针旋转
- 指定角度旋转
- 持续旋转
- 完整旋转机动（起飞→悬停→旋转→悬停→降落）

### 6. 状态回传
- 高度、速度、电量、模式
- 航向、姿态（pitch/roll）
- 信号强度等

---

## 已创建的文件

### 1. drone_server.py
**位置**: 项目目录
**功能**: 香橙派服务端程序

**主要类和方法**:
```python
class DroneServer:
    def __init__(self, pc_ip='192.168.1.50')
    def start(self, connection_string='/dev/ttyACM0')
    def connect_vehicle(self, connection_string)
    def arm_and_takeoff(self, altitude)
    def set_rc_override(self, roll, pitch, throttle, yaw, duration=None)
    def yaw_control(self, degrees, direction, speed)
    def idle_test(self, duration=5)
    def system_check(self)
    def emergency_stop()
    def broadcast_telemetry()
    def execute_rotation_maneuver(self, altitude, angle, direction, speed=10)
```

### 2. ground_station.py
**位置**: 项目目录
**功能**: PC地面站控制程序

**主要类和方法**:
```python
class GroundStation:
    def __init__(self, orange_pi_ip='192.168.1.100')
    def connect()
    def send_command(cmd_type, **params)
    def arm_and_takeoff(altitude=2)
    def land()
    def idle_test(duration=5)
    def system_check()
    def yaw(degrees=10, direction=1, speed=10)
    def emergency_stop()
```

### 3. yaw_controller.py
**位置**: 项目目录（待创建）
**功能**: 转向控制扩展模块

**主要方法**:
```python
def rotate_continuous(direction=1, speed=10, duration=None)
def rotate_angle(angle, direction=1, speed=15)
def execute_rotation_maneuver(altitude, angle, direction, speed=10)
```

### 4. wifi_connect.sh
**位置**: 香橙派
**功能**: WiFi自动连接脚本

---

## 指令格式

### PC→香橙派 控制指令 (JSON over TCP)
```json
{
    "type": "arm_and_takeoff",
    "params": {"altitude": 2}
}
```

### 香橙派→PC 状态回传 (JSON over UDP)
```json
{
    "type": "telemetry",
    "data": {
        "altitude": 2.5,
        "speed": 0.5,
        "battery": 85,
        "mode": "GUIDED",
        "armed": true,
        "heading": 180,
        "pitch": 0.02,
        "roll": 0.01
    },
    "timestamp": 1673456789.5
}
```

---

## 使用步骤

### 香橙派端

```bash
# 1. 连接WiFi
chmod +x wifi_connect.sh
./wifi_connect.sh

# 2. 查看IP
ip addr show wlan0
# 假设获取到 192.168.1.100

# 3. 启动服务
python3 drone_server.py --pc-ip 192.168.1.50 --connection /dev/ttyACM0
```

### PC端

```python
from ground_station import GroundStation

gs = GroundStation('192.168.1.100')  # 香橙派IP
gs.connect()

# 指令序列
gs.system_check()      # 系统自检
gs.idle_test(5)        # 怠速测试5秒
gs.arm_and_takeoff(2)  # 起飞到2米
gs.yaw(90, 1)          # 顺时针转90度
gs.land()              # 降落
```

---

## 关键代码片段

### 怠速测试实现 (参考noGPS.py)
```python
def idle_test(self, duration=5):
    # STABILIZE模式
    self.vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(1)

    # 解锁
    self.vehicle.armed = True
    time.sleep(2)

    # 怠速油门 1050-1250
    throttle = 1050
    direction = 1
    throttle_step = 5

    start = time.time()
    while time.time() - start < duration:
        self.vehicle.channels.overrides = {'3': throttle}
        throttle += direction * throttle_step
        if throttle >= 1250:
            direction = -1
        elif throttle <= 1050:
            direction = 1
        time.sleep(0.1)

    # 停止
    self.vehicle.channels.overrides = {}
    self.vehicle.armed = False
```

### 转向控制
```python
def yaw_control(self, degrees, direction, speed=10):
    """direction: 1=顺时针, -1=逆时针"""
    msg = self.vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, degrees, speed, direction, 1, 0, 0, 0
    )
    self.vehicle.send_mavlink(msg)
```

---

## 注意事项

1. **IP配置**: 香橙派IP由路由器DHCP分配，保持不变
2. **遥控器**: 有遥控器可手动接管控制
3. **仿真测试**: 可先在Ubuntu SITL环境测试
4. **文件位置**: 代码文件在项目目录，脚本需上传到香橙派

---

## 待完成任务

- [ ] 创建 drone_server.py 完整代码
- [ ] 创建 ground_station.py 完整代码
- [ ] 创建 wifi_connect.sh 脚本
- [ ] 创建 yaw_controller.py 扩展模块
- [ ] 测试SITL仿真环境
- [ ] 实机测试

---

## 回传数据及处理

| 数据 | 字段 | 说明 | 处理动作 |
|-----|------|------|---------|
| 电量 | battery.level | <20%返航, <10%紧急降落 |
| 高度 | altitude | >100m限高 |
| 姿态 | pitch/roll | pitch>45°危险 |
| 信号 | rssi | <20%失控保护 |
| 模式 | mode.name | 状态切换 |

---

*本文档由AI助手生成，用于记录项目上下文*
