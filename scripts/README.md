# Scripts (脚本)

这里保留的是**启动/环境辅助脚本**，不再维护与 `tests/` 重复的飞行测试逻辑。

## 文件说明

| 文件 | 功能描述 |
| --- | --- |
| `run.sh` | 项目运行入口，根据参数启动地面站、无人机端、机器狗端或本地模拟。 |
| `wifi_connect.sh` | 香橙派侧 WiFi 管理脚本，支持扫描、自动连接、开机自启、状态回传。 |
| `ubuntu_wifi_connect.sh` | Ubuntu PC 侧 WiFi 管理脚本，支持扫描、自动连接、开机自启。 |

## 使用方法

### 启动系统

```bash
# 默认启动地面站
bash scripts/run.sh

# 启动无人机端
bash scripts/run.sh --drone --connection /dev/ttyACM0

# 启动机器狗端
bash scripts/run.sh --dog

# 启动本地模拟
bash scripts/run.sh --sim
```

### WiFi 脚本

```bash
# 香橙派端
sudo bash scripts/wifi_connect.sh --auto
sudo bash scripts/wifi_connect.sh --scan
sudo bash scripts/wifi_connect.sh --autostart

# Ubuntu PC 端
sudo bash scripts/ubuntu_wifi_connect.sh --auto
sudo bash scripts/ubuntu_wifi_connect.sh --scan
```

## 说明

- 原来的 shell 版飞行测试脚本已移除，统一以 `tests/` 下的 Python 测试脚本为准。
- `run.sh` 需要在项目根目录执行，脚本内部会设置 `PYTHONPATH`。
- WiFi 脚本需要 `sudo` 权限。
