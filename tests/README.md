# Tests (测试代码)

> scripts目录下的测试仅限于测试阶段使用,后期的脚本是python格式,使用python控制代码飞行,当我们通过了SITL测试后即可迁移到简单的python脚本测试
>   uav/tests/flight_test.py:各个选项的功能是一样的 直接full即可
>   uav/tests/uav_test.py:其中的SITL功能是多余的;硬件pix6测试也是多余的;也可以不运行scripts目录下的测试 直接使用该脚本进行测试;

各种测试脚本，用于验证飞控连接、功能测试和诊断。

## 文件说明

| 文件 | 功能描述 |
|-----|---------|
| `flight_test.py` | **综合飞行测试**。SITL仿真中测试起飞→圆形飞行→悬停→返航完整流程。包含解锁检查、姿态监控、航点飞行等。 |
| `sitl_control.py` | SITL模拟器启动脚本。自动检测SITL进程，启动ArduCopter仿真，记录日志到/tmp/sitl.log。 |
| `mavlink_test.py` | MAVLink基础测试。连接飞控，测试起飞、速度控制、姿态控制等基本MAVLink指令。 |
| `ahrs_diagnostics.py` | **AHRS诊断程序**。检测和解决"Bad AHRS"及无法解锁电机的问题，提供系统状态检查和解决方案。 |
| `pix6_controller.py` | Pix6飞控无RC控制模块。禁用RC检测和保护，方便纯代码控制飞控。用于室内无遥控器环境。 |
| `pix6_diagnostics.py` | Pix6飞控诊断工具。连接并获取所有参数和遥测数据，验证是否获取真实信息。 |
| `uav_test.py` | **UAV完整测试套件**。先在SITL中测试，再在Pix6硬件上测试。支持自动模式和交互模式，记录测试结果。 |

## 运行方式

```bash
# SITL飞行测试
python3 tests/flight_test.py --auto

# AHRS诊断
python3 tests/ahrs_diagnostics.py --connection /dev/ttyACM0

# Pix6诊断
python3 tests/pix6_diagnostics.py

# 完整测试套件（SITL模式）
python3 tests/uav_test.py --sitl --auto

# 完整测试套件（Pix6硬件）
python3 tests/uav_test.py --pix6 --auto
```

## 测试流程 (uav_test.py)

```
1. 连接测试 → 飞控连接验证
2. 解锁测试 → 检查是否可以解锁
3. 起飞测试 → 起飞到指定高度
4. 悬停测试 → 悬停指定时间
5. 姿态测试 → 检查pitch/roll/yaw
6. 降落测试 → 安全降落
```

## 常见问题排查

| 问题 | 解决方案 |
|-----|---------|
| 无法连接飞控 | 检查USB端口、波特率、权限 |
| Bad AHRS | 运行 `ahrs_diagnostics.py` |
| 无法解锁 | 检查遥控器连接、飞控参数 |
| SITL无响应 | 运行 `sitl_control.py` 重启 |
