# UAV 项目目录说明

这是一个同时包含**真实设备控制**、**测试脚本**、**路径规划仿真**和**历史实验代码**的仓库。当前目录已经按“可运行主线 / 支撑配置 / 仿真研究 / 历史归档”四层来理解会最清晰。

## 当前主线

- 真实运行主线在 `src/`：无人机端、地面站、机器狗适配器、公用通信模块都在这里。
- 验证与诊断主线在 `tests/`：这里保留 Python 测试脚本，作为目前推荐的测试入口。
- 环境与启动辅助在 `scripts/`：这里只放运行入口和 WiFi/部署脚本，不再放重复的飞行测试实现。
- 路径规划研究在 `simulation/`：独立于真实飞控控制链，主要用于算法验证、场景配置和结果输出。

## 顶层目录作用

| 目录 | 作用 | 备注 |
| --- | --- | --- |
| `src/` | 项目核心源码 | 真实设备控制与通信主线 |
| `tests/` | Python 测试与诊断 | 推荐优先从这里跑飞控验证 |
| `scripts/` | 启动与环境脚本 | 偏运维，不承载核心业务逻辑 |
| `config/` | 飞控参数和网络配置 | `.parm` 为参数集，`network_config.json` 为网络配置 |
| `simulation/` | 路径规划仿真与实验输出 | 包含 2D/3D 算法、场景、评估、可视化 |
| `docs/` | 项目文档 | 学习资料、阶段方案、FAQ、上下文 |
| `archive/` | 历史实验代码 | 仅保留有参考价值的旧脚本 |

## `src/` 子目录说明

| 子目录 | 作用 | 关键文件 |
| --- | --- | --- |
| `src/common/` | 公共协议、TCP 封装、心跳、日志、配置管理 | `protocol.py`, `tcp_base.py`, `heartbeat.py`, `config_manager.py` |
| `src/drone/` | 无人机端控制逻辑 | `drone_server.py`, `yaw_controller.py`, `fly.py`, `noGPS.py` |
| `src/ground_station/` | PC 地面站控制端 | `ground_station.py` |
| `src/dog/` | 机器狗动作适配层 | `motion_adapter.py` |

## `simulation/` 子目录说明

| 子目录 | 作用 |
| --- | --- |
| `simulation/algorithm/` | 路径规划与路径优化算法 |
| `simulation/3d/` | 3D UAV 路径规划与可视化 |
| `simulation/visualization/` | 2D 对比可视化 |
| `simulation/evaluation/` | 路径指标评估 |
| `simulation/config/` | 仿真场景配置 |
| `simulation/output/` | 已生成的图表与报告 |
| `simulation/docs/` | 仿真阶段的专题文档 |

## 推荐入口

### 真实控制

```bash
# 地面站
python3 src/ground_station/ground_station.py

# 无人机端服务
python3 src/drone/drone_server.py --connection /dev/ttyACM0
```

### 测试与诊断

```bash
# SITL/飞行流程测试
python3 tests/flight_test.py --mode full

# 综合测试入口
python3 tests/uav_test.py --mode sitl --auto

# AHRS 诊断
python3 tests/ahrs_diagnostics.py --connection /dev/ttyACM0
```

### 仿真

```bash
# 路径融合主程序
python3 simulation/main_path_fusion.py
```

## 这次整理做了什么

- 删除 `scripts/uav_test.sh`：与 `tests/` 下 Python 测试脚本职责重叠，维护成本高。
- 删除 `archive/a.py`：与 `archive/lati_hold_fly.py` 高度重复，且命名无语义。
- 删除 `archive/data/simulate.py`：仓库内已标注为“暂时不需要，可丢弃”的旧本地模拟代码。
- 保留 `archive/` 里仍有参考价值的硬件实验脚本，避免一次性误删全部历史资料。

## 阅读顺序建议

1. 先看 `docs/LEARNING_GUIDE.md`
2. 再看 `src/common/` 与 `src/drone/`
3. 然后看 `tests/` 的诊断与飞行测试脚本
4. 最后再看 `simulation/` 的算法实验链路
