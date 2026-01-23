# SITL模拟器测试指南

本文档说明如何在本地使用SITL模拟器测试无人机代码。

---

## 1. 启动SITL模拟器

在终端1中启动SITL：

```bash
cd /home/undefined/Desktop/uav/ardupilot
./build/sitl/bin/arducopter --model +
```

启动成功后看到以下输出表示正常：
```
bind port 5760 for SERIAL0
SERIAL0 on TCP port 5760
Home: -35.363262 149.165237 alt=584.000000m hdg=353.000000
```

---

## 2. 启动MAVProxy（地图+控制台）

在终端2中启动MAVProxy：

```bash
mavproxy.py --master=tcp:127.0.0.1:5760 --map --console
```

这会弹出两个窗口：
- **地图窗口**：显示飞机位置和飞行轨迹
- **控制台窗口**：显示飞行数据和指令

---

## 3. 测试转向控制器

在终端3中运行转向控制器测试：

```bash
cd /home/undefined/Desktop/uav/uav

# 自动测试（旋转45度）
python3 yaw_controller.py --auto

# 自动测试（自定义角度）
python3 yaw_controller.py --auto --angle 30

# 交互式测试（可选择不同测试项目）
python3 yaw_controller.py
```

**自动测试输出示例：**
```
============================================================
   转向控制器测试程序
============================================================
连接: tcp:127.0.0.1:5760 @ 57600
------------------------------------------------------------
✓ 飞控连接成功!
  模式: STABILIZE
  姿态: pitch=0.00, roll=0.00, yaw=-0.10
------------------------------------------------------------

[自动测试模式]
------------------------------------------------------------

[1] 查看当前姿态
  Pitch: 0.00°
  Roll:  0.00°
  Yaw:   -0.10°

[2] 旋转30.0度（顺时针）
旋转: 30.0度, 顺时针, 速度=15度/秒
旋转完成

[3] 旋转30.0度（逆时针）
旋转: 30.0度, 逆时针, 速度=15度/秒
旋转完成

[4] 当前航向: 354°

============================================================
自动测试完成!
============================================================
```

---

## 4. 测试怠速功能

在终端3中运行：

```bash
cd /home/undefined/Desktop/uav/uav
python3 drone_server.py --connection tcp:127.0.0.1:5760 --baud 57600 --port 5000
```

然后可以用ground_station.py连接测试。

---

## 5. 连接参数说明

| 场景 | 连接字符串 | 波特率 |
|-----|----------|--------|
| **SITL模拟器** | `tcp:127.0.0.1:5760` | 57600 |
| **真实飞控(USB)** | `/dev/ttyACM0` | 921600 |
| **真实飞控(串口)** | `/dev/ttyS0` | 57600 |

---

## 6. 常用命令

### 停止SITL
```bash
pkill -9 -f arducopter
```

### 检查SITL状态
```bash
ss -tlnp | grep 5760
```

### 查看SITL日志
```bash
tail -20 /tmp/sitl.log
```

---

## 7. 目录结构

```
/home/undefined/Desktop/uav/
├── ardupilot/
│   ├── build/sitl/bin/arducopter    # SITL二进制文件
│   └── Tools/autotest/sim_vehicle.py # 模拟器启动脚本
│
└── uav/
    ├── yaw_controller.py            # 转向控制器（带测试）
    ├── drone_server.py              # 香橙派服务端
    ├── ground_station.py            # PC地面站
    ├── ubuntu_wifi_connect.sh       # PC WiFi连接脚本
    ├── wifi_connect.sh              # 香橙派WiFi连接脚本
    └── SITL测试指南.md              # 本文档
```

---

## 8. 注意事项

1. **SITL必须在Python代码之前启动**
2. **连接SITL使用TCP端口5760**，而不是UDP 14550
3. 怠速测试会真正启动电机，模拟测试时请勿安装浆叶
4. MAVProxy地图窗口需要GUI环境（不能在纯终端中运行）
