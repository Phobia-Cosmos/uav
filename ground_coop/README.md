# Ground-Air Cooperation System

地空协同系统 - 无人机与机器狗协调控制。

## 目录结构

```
ground_coop/
├── config/
│   └── network_config.json     # 网络配置（含WiFi-IP映射）
├── common/
│   ├── __init__.py
│   ├── config_manager.py       # 配置管理（WiFi智能选择）
│   ├── logger.py               # 双位置日志（文件+journalctl）
│   ├── protocol.py             # JSON消息协议定义
│   ├── heartbeat.py            # 心跳检测模块
│   └── tcp_base.py             # TCP客户端/服务端基类
├── ground_station/             # PC端 (192.168.55.126)
│   └── main.py                 # 地面站主程序
├── drone/                      # 无人机端 (192.168.55.128)
│   ├── main.py                 # 无人机主程序
│   ├── flight_controller.py    # 飞控控制（dronekit）
│   └── dog_commander.py        # 机器狗通信
├── dog/                        # 机器狗端 (192.168.55.127)
│   ├── main.py                 # 机器狗主程序
│   └── motion_adapter.py       # ROS服务适配器
├── requirements.txt
└── README.md
```

## 快速开始

### 安装依赖

```bash
pip install -r requirements.txt
```

### 配置网络

编辑 `config/network_config.json`，根据当前连接的WiFi设置对应IP：

```json
{
    "wifi_profiles": {
        "2楼": {
            "pc": {"ip": "192.168.55.126", ...},
            "drone": {"ip": "192.168.55.128", ...},
            "dog": {"ip": "192.168.55.127", ...}
        },
        "Undefined": {
            "pc": {"ip": "X.X.X.X", ...},
            ...
        }
    }
}
```

### 运行程序

#### PC端（地面站）
```bash
cd ground_coop
python ground_station/main.py --auto
```

#### 无人机端
```bash
cd ground_coop
python drone/main.py --auto
```

#### 机器狗端
```bash
cd ground_coop
python dog/main.py --auto
```

## 命令说明

### 地面站命令

| 命令 | 说明 |
|-----|------|
| `start [altitude]` | 启动无人机起飞（默认3米） |
| `stop` | 停止并返航 |
| `control` | 进入控制模式（无人机控制机器狗） |
| `status` | 查看当前状态 |
| `quit` | 退出程序 |

## 通信架构

```
┌─────────────────────────────────────────────────────────────┐
│                      192.168.55.0/24 网段                     │
├─────────────────────────────────────────────────────────────┤
│  PC (5100/5101)  ←──TCP──→  无人机 (5200/5300)  ←──TCP──→  机器狗 (5400) │
└─────────────────────────────────────────────────────────────┘

端口分配：
- 5100: PC接收无人机状态
- 5101: PC接收机器狗状态
- 5200: 无人机接收PC命令
- 5300: 无人机与机器狗通信
- 5400: 机器狗接收无人机命令
```

## 心跳机制

- 发送间隔: 10秒
- 超时时间: 30秒
- 超时处理: 无人机悬停→返航，机器狗停止

## 日志

日志同时输出到：
- 文件: `/tmp/ground_coop.log`
- journalctl: `journalctl -u ground-coop-*`

## 开机自启

创建systemd服务（需要root权限）：

```bash
# 无人机端
sudo cp deploy/ground-coop-drone.service /etc/systemd/system/
sudo systemctl enable ground-coop-drone
sudo systemctl start ground-coop-drone

# 机器狗端
sudo cp deploy/ground-coop-dog.service /etc/systemd/system/
sudo systemctl enable ground-coop-dog
sudo systemctl start ground-coop-dog
```
