# UAV代码学习指南

## 目录结构（精简版）

```
uav/
├── core/                          # 核心代码（必学）
│   ├── drone_server.py            # 飞控连接与服务端
│   └── ground_station.py          # PC地面站控制
├── features/                      # 功能模块（进阶）
│   ├── yaw_controller.py          # 航向控制
│   ├── flight_test.py             # 完整飞行测试
│   └── ahrs_diagnostics.py        # AHRS诊断工具
├── tools/                         # 工具脚本
│   ├── sitl_control.py            # SITL模拟器控制
│   └── SITL测试指南.md            # SITL使用文档
├── archive/                       # 历史代码（参考）
└── README.md                      # 项目说明
```

---

## 学习路径（建议顺序）

### 阶段1：基础连接（1-2天）

| 顺序 | 文件 | 重点内容 | 学习目标 |
|-----|------|---------|---------|
| 1 | `drone_server.py` | `connect_vehicle()` | 理解如何连接飞控 |
| 2 | `ground_station.py` | `GroundStation`类 | 理解指令发送流程 |

**核心概念：**
```python
# 连接飞控
vehicle = connect(connection_string, wait_ready=True, baud=baud)

# 获取状态
print(vehicle.mode.name)        # 飞行模式
print(vehicle.armed)            # 解锁状态
print(vehicle.attitude)         # 姿态数据
```

---

### 阶段2：解锁与起飞（2-3天）

| 顺序 | 文件 | 重点内容 | 学习目标 |
|-----|------|---------|---------|
| 1 | `flight_test.py` | `arm_and_takeoff()` | 掌握标准解锁流程 |
| 2 | `drone_server.py` | `idle_test()` | 理解怠速测试 |

**解锁流程：**
```python
# 1. 等待飞控就绪
while not vehicle.is_armable:
    time.sleep(1)

# 2. 切换模式
vehicle.mode = VehicleMode("GUIDED")

# 3. 解锁
vehicle.armed = True

# 4. 起飞
vehicle.simple_takeoff(target_altitude)
```

---

### 阶段3：飞行控制（3-4天）

| 顺序 | 文件 | 重点内容 | 学习目标 |
|-----|------|---------|---------|
| 1 | `yaw_controller.py` | `condition_yaw` | 航向控制原理 |
| 2 | `flight_test.py` | `send_local_ned_velocity()` | 速度控制 |
| 3 | `drone_server.py` | `set_rc_override()` | RC覆盖控制 |

**关键指令：**
```python
# 航向控制（MAV_CMD_CONDITION_YAW）
msg = vehicle.message_factory.command_long_encode(
    0, 0,
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
    0, angle, speed, direction, is_relative, 0, 0, 0
)

# 速度控制（NED坐标系）
msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0, 0, 0,
    mavutil.mavlink.MAV_FRAME_BODY_NED,
    0b0000111111000111,
    0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
)
```

---

### 阶段4：传感器与诊断（2天）

| 顺序 | 文件 | 重点内容 |
|-----|------|---------|
| 1 | `ahrs_diagnostics.py` | AHRS/IMU检查 |
| 2 | `flight_test.py` | `arm_check()` |

**诊断要点：**
- `vehicle.is_armable` - 飞控可解锁状态
- `vehicle.attitude` - 姿态(pitch/roll/yaw)
- `vehicle.battery` - 电池电压
- `system_status.state` - 系统状态

---

### 阶段5：高级功能（可选）

| 文件 | 功能 |
|-----|------|
| `sitl_control.py` | SITL模拟器管理 |
| `SITL测试指南.md` | 仿真环境搭建 |

---

## 常见问题速查

| 问题 | 原因 | 解决 |
|-----|------|-----|
| 无法解锁 | `is_armable=False` | 运行`ahrs_diagnostics.py`诊断 |
| Bad AHRS | IMU未校准/飞控不平 | 重新校准加速度计 |
| 连接失败 | 端口/波特率错误 | 检查`/dev/ttyACM0`和`921600` |
| 姿态异常 | 角度>45° | 确保飞控水平放置 |

---

## 推荐学习顺序

```
1. 运行一次 ahrs_diagnostics.py 了解飞控状态
2. 用 flight_test.py --mode arm_test 测试解锁
3. 用 sitl_control.py 启动SITL仿真
4. 逐步阅读 drone_server.py 核心代码
5. 修改 ground_station.py 添加自定义功能
```

---

## 文件说明速查

| 文件 | 功能 | 代码行数 |
|-----|------|---------|
| `drone_server.py` | 香橙派服务端 | 685 |
| `ground_station.py` | PC地面站 | 356 |
| `flight_test.py` | 飞行测试 | 391 |
| `yaw_controller.py` | 航向控制 | 275 |
| `ahrs_diagnostics.py` | 诊断工具 | 310 |
| `sitl_control.py` | SITL控制 | 104 |

---

*代码学习路径由AI助手生成，建议配合ArduPilot官方文档学习*
