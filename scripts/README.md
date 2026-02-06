# Scripts (脚本)

> uav_test.sh: 开发流程为写飞行脚本代码 → SITL测试 → 真机实飞；首先要先通过SITL测试，该脚本用于简单功能的测试，如果可以通过我们再测试自己编写的复杂的代码；但是后续的脚本代码测试需要新的脚本测试，封装完整性需要提升
> run.sh: 空地协同救援系统一键测试脚本

## 文件说明

| 文件 | 功能描述 |
|-----|---------|
| `run.sh` | **系统启动器**。地空协同系统主入口脚本，通过参数选择运行地面站、无人机、机器狗或本地模拟。自动设置PYTHONPATH。 |
| `uav_test.sh` | **UAV一键测试脚本**。完整的飞行测试套件，支持SITL仿真和Pix6硬件两种模式。包含连接测试、模式切换、解锁/锁定、遥测数据、起飞测试、绕圈飞行、方形飞行、返回降落、完整飞行测试等9项测试。 |
| `ubuntu_wifi_connect.sh` | **Ubuntu WiFi连接**。PC端WiFi管理脚本，支持多网络配置（i-HDU、Undefined、2楼），支持开机自启、扫描网络、查看IP等功能。 |
| `wifi_connect.sh` | **香橙派WiFi连接**。香橙派端WiFi管理脚本，功能与Ubuntu版本类似，但会向PC回传WiFi状态，支持开机自启。 |

## 使用方法

### 系统启动 (run.sh)

```bash
# 运行地面站（默认）
bash scripts/run.sh

# 运行无人机端（需要连接飞控）
bash scripts/run.sh --drone --fc-connection /dev/ttyACM0

# 运行机器狗端
bash scripts/run.sh --dog

# 运行本地模拟
bash scripts/run.sh --sim
```

### UAV测试 (uav_test.sh)

```bash
# SITL自动完整测试
bash scripts/uav_test.sh --sitl --auto

# Pix6硬件自动完整测试
bash scripts/uav_test.sh --pix6 --auto

# SITL交互式测试（可选择单项测试）
bash scripts/uav_test.sh --sitl

# 停止所有进程
bash scripts/uav_test.sh --stop
```

测试项目：
1. 飞控连接
2. 模式切换 (STABILIZE/ALT_HOLD/GUIDED)
3. 解锁/锁定
4. 遥测数据
5. 起飞测试 (到10米)
6. 绕圈飞行 (20米半径)
7. 方形飞行 (20x20米)
8. 返回降落
9. 完整飞行测试

### WiFi连接

```bash
# 香橙派端
sudo bash scripts/wifi_connect.sh --auto        # 自动连接
sudo bash scripts/wifi_connect.sh --scan        # 扫描网络
sudo bash scripts/wifi_connect.sh --autostart   # 开机自启

# Ubuntu PC端
sudo bash scripts/ubuntu_wifi_connect.sh --auto
sudo bash scripts/ubuntu_wifi_connect.sh --scan
```

## 注意事项

1. 脚本需要执行权限：`chmod +x scripts/*.sh`
2. WiFi脚本需要root权限：`sudo`
3. `run.sh` 需要在项目根目录运行（自动设置PYTHONPATH）
4. `uav_test.sh` 需要ArduPilot环境（路径在脚本中配置）
