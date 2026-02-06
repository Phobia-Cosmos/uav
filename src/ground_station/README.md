# Ground Station Module (PC地面站)

PC端地面站程序，运行在控制电脑上，负责用户交互和发送控制指令。

## 文件说明

| 文件 | 功能描述 |
|-----|---------|
| `ground_station.py` | **核心程序**。TCP客户端，连接香橙派服务端，发送控制指令（起飞、降落、返航、转向等），并显示无人机状态信息。 |

## 核心类

### GroundStation

```python
class GroundStation:
    def __init__(self, server_ip, server_port)
    def connect(self)                    # 连接到香橙派
    def disconnect(self)                 # 断开连接
    def send_command(self, type, **params)  # 发送指令
    def arm_and_takeoff(self, altitude)  # 起飞
    def land(self)                       # 降落
    def return_home(self)                # 返航
    def yaw(self, degrees, direction)   # 转向
    def idle_test(self, duration)        # 怠速测试
    def emergency_stop(self)             # 紧急停止
    def get_status(self)                 # 获取状态
```

## 指令格式

```json
{
    "type": "command_type",
    "params": {}
}
```

支持的指令类型：
- `arm_and_takeoff`: 起飞到指定高度
- `land`: 降落
- `return`: 返航
- `yaw`: 转向
- `idle_test`: 怠速测试
- `emergency_stop`: 紧急停止

## 运行方式

```bash
# 启动地面站
python3 src/ground_station/ground_station.py --ip 192.168.1.100

# 进入交互模式后：
# start [altitude]  - 起飞
# land              - 降落
# return            - 返航
# yaw [degrees]     - 转向
# quit              - 退出
```

## 状态回传

香橙派会定期回传以下状态：

```json
{
    "altitude": 2.5,
    "speed": 0.5,
    "battery": 85,
    "mode": "GUIDED",
    "armed": true,
    "latitude": 0.0,
    "longitude": 0.0,
    "pitch": 0.02,
    "roll": 0.01,
    "yaw": 180
}
```
