# UAV实时控制系统

> 空地协同系统 - 无人机与机器狗协调控制
> archive/data目录下的模拟通信暂时不需要 可以后续考虑或者丢弃

## 项目结构

```
uav/
├── src/                        # 源代码
│   ├── drone/                 # 无人机端
│   │   ├── drone_server.py    # 香橙派服务端（核心）
│   │   ├── fly.py             # 飞行控制主程序
│   │   ├── yaw_controller.py   # 航向控制
│   │   └── noGPS.py           # 无GPS飞行
│   │
│   ├── ground_station/        # PC地面站
│   │   └── ground_station.py  # 地面站控制程序
│   │
│   ├── dog/                   # 机器狗端
│   │   └── motion_adapter.py  # ROS服务适配器
│   │
│   └── common/                # 公共模块（保留原ground_coop/common）
│       ├── protocol.py        # JSON消息协议
│       ├── tcp_base.py        # TCP客户端/服务端基类
│       ├── heartbeat.py       # 心跳检测
│       ├── config_manager.py  # 配置管理
│       └── logger.py          # 日志
│
├── tests/                     # 测试代码
│   ├── flight_test.py         # 飞行测试
│   ├── mavlink_test.py        # MAVLink测试
│   ├── sitl_control.py        # SITL仿真控制
│   ├── ahrs_diagnostics.py   # AHRS诊断
│   ├── pix6_controller.py     # PIX6飞控
│   ├── pix6_diagnostics.py    # PIX6诊断
│   └── uav_test.py            # UAV综合测试
│
├── scripts/                   # 脚本
│   ├── wifi_connect.sh        # WiFi自动连接
│   ├── ubuntu_wifi_connect.sh # Ubuntu WiFi连接
│   ├── sitl_start.sh          # SITL启动
│   ├── sitl_quick_start.sh   # SITL快速启动
│   ├── uav_test.sh           # UAV测试脚本
│   └── run.sh                # 运行脚本
│
├── config/                    # 配置文件
│   ├── frame_config.parm      # 飞控参数
│   ├── quad_config.parm       # 四旋翼参数
│   ├── quad_setup.parm        # 四旋翼设置
│   ├── mav.parm              # MAVLink参数
│   └── network_config.json    # 网络配置
│
├── docs/                      # 文档
│   ├── README.md              # 项目说明
│   ├── PROJECT_CONTEXT.md     # 项目上下文
│   ├── LEARNING_GUIDE.md      # 学习指南
│   ├── FAQ.md                # 常见问题
│   └── SITL测试指南.md        # SITL测试文档
│
├── archive/                   # 历史代码
│   ├── cv2_test.py           # OpenCV测试
│   ├── distance.py           # 超声波测距
│   ├── lati_hold_fly.py      # 纬度保持飞行
│   ├── example1.py           # 示例代码
│   ├── a.py                 # 旧代码
│   └── data/                 # 模拟数据（暂时不需要，可丢弃）
│       └── simulate.py
│
├── logs/                      # 日志文件
│   ├── mav.tlog              # MAVLink日志
│   └── mav.tlog.raw          # MAVLink原始日志
│
├── requirements.txt           # Python依赖
└── .gitignore                # Git忽略配置
```

## 快速开始

```bash
# 1. 安装依赖
pip install -r requirements.txt

# 2. 启动SITL仿真
python3 scripts/sitl_start.sh

# 3. 运行测试
python3 tests/flight_test.py --mode arm_test
```

## 通信架构

```
PC (192.168.55.126) ←──TCP──→ 无人机 (192.168.55.128) ←──TCP──→ 机器狗 (192.168.55.127)

端口分配：
- PC: 5100/5101
- 无人机: 5200/5300
- 机器狗: 5400
```

## 文档

- [学习指南](docs/LEARNING_GUIDE.md) ← 推荐先读这个
- [项目上下文](docs/PROJECT_CONTEXT.md)
- [FAQ](docs/FAQ.md)
- [SITL测试指南](docs/SITL测试指南.md)
