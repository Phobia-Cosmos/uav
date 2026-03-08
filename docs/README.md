# UAV 文档目录

这里存放项目背景、学习路径、阶段设计和常见问题文档。

## 推荐阅读顺序

1. `docs/LEARNING_GUIDE.md`
2. `docs/PROJECT_CONTEXT.md`
3. `docs/FAQ.md`
4. `docs/PHASE1_IMPLEMENTATION.md` 到 `docs/PHASE4_PLAN.md`

## 快速开始

```bash
# 1. 先做 AHRS / PreArm 诊断
python3 tests/ahrs_diagnostics.py --connection /dev/ttyACM0

# 2. 跑一遍 SITL 飞行流程测试
python3 tests/flight_test.py --mode full

# 3. 需要分项定位问题时
python3 tests/uav_test.py --mode sitl --auto
```

## 当前代码主线

| 目录 | 说明 |
| --- | --- |
| `src/drone/` | 无人机端真实控制代码 |
| `src/ground_station/` | 地面站控制端 |
| `src/dog/` | 机器狗动作适配层 |
| `tests/` | 诊断与飞行测试入口 |
| `simulation/` | 路径规划仿真与评估 |

## 连接方式

| 环境 | 连接字符串 | 波特率 |
| --- | --- | --- |
| SITL 仿真 | `udp:127.0.0.1:14550` | 57600 |
| 真实飞控 (USB) | `/dev/ttyACM0` | 921600 |

## 常用端口

- `TCP 5000`：地面站到无人机控制指令
- `TCP 5001`：状态回传 / 状态监听
- `UDP 14550`：SITL / MAVLink 常用端口
