# UAV实时控制系统

PC端地面站实时控制无人机的完整解决方案。

## 快速开始

```bash
# 1. 启动SITL仿真
python3 sitl_control.py

# 2. 运行诊断（检测Bad AHRS）
python3 ahrs_diagnostics.py --connection tcp:127.0.0.1:5760 --baud 57600

# 3. 飞行测试
python3 flight_test.py --mode arm_test
```

## 项目结构

```
uav/
├── drone_server.py          # 香橙派服务端（核心）
├── ground_station.py        # PC地面站
├── flight_test.py           # 飞行测试
├── yaw_controller.py        # 航向控制
├── ahrs_diagnostics.py      # AHRS诊断
├── sitl_control.py          # SITL模拟器
├── LEARNING_GUIDE.md        # 学习指南 ← 推荐先读这个
└── archive/                 # 历史代码
```

## 连接方式

| 环境 | 连接字符串 | 波特率 |
|-----|----------|--------|
| SITL仿真 | `tcp:127.0.0.1:5760` | 57600 |
| 真实飞控(USB) | `/dev/ttyACM0` | 921600 |

## 学习路径

见 `LEARNING_GUIDE.md`

## 通信端口

- TCP 5000: 控制指令
- TCP 5001: 状态回传
- UDP 14550: MAVLink
