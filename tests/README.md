# Tests (测试代码)

这里是**真实飞控 / 真机环境**的测试与诊断入口，不属于当前 `simulation/` 仿真主线。

与历史 shell 测试脚本相比，`tests/` 下的 Python 脚本更集中、可复用，也更适合作为真实环境验证基线。

## 文件说明

| 文件 | 功能描述 |
| --- | --- |
| `flight_test.py` | SITL 飞行流程测试，覆盖起飞、圆形航迹、悬停、返航等流程。 |
| `uav_test.py` | 综合测试套件，适合按测试项分步验证连接、模式、解锁、起飞、移动、降落、遥测。 |
| `mavlink_test.py` | MAVLink 基础控制测试，适合验证速度、姿态、偏航等底层控制指令。 |
| `ahrs_diagnostics.py` | AHRS / PreArm / 传感器健康诊断。 |
| `pix6_controller.py` | Pix6 纯代码控制辅助模块，适合室内无遥控器场景。 |
| `pix6_diagnostics.py` | Pix6 参数和遥测信息诊断脚本。 |

## 推荐运行顺序

```bash
# 1. SITL 验证主流程
python3 tests/flight_test.py --mode full

# 2. 分项综合测试
python3 tests/uav_test.py --mode sitl --auto

# 3. 真机出现解锁/姿态异常时诊断
python3 tests/ahrs_diagnostics.py --connection /dev/ttyACM0
```

## 常见问题

| 问题 | 处理方式 |
| --- | --- |
| 无法连接飞控 | 检查连接串、波特率、串口权限 |
| 无法解锁 | 先跑 `ahrs_diagnostics.py` 查看 PreArm / AHRS 状态 |
| SITL 无响应 | 检查模拟器端口是否正确、是否已有旧进程占用 |

## 说明

- 旧的 shell 版飞行测试脚本已移除，避免与这里的 Python 测试逻辑重复维护。
- 如果只想快速验证完整飞行流程，优先使用 `flight_test.py`。
- 如果想分步骤定位问题，优先使用 `uav_test.py`。
