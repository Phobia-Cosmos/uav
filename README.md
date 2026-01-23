# UAV实时控制系统

PC端地面站实时控制无人机的完整解决方案。

---

## 项目结构

```
uav/
├── drone_server.py      # 香橙派服务端
├── ground_station.py    # PC地面站控制
├── yaw_controller.py    # 转向控制模块
├── ubuntu_wifi_connect.sh   # Ubuntu PC WiFi连接脚本
├── wifi_connect.sh      # 香橙派WiFi连接脚本
├── PROJECT_CONTEXT.md   # 项目上下文文档
└── README.md            # 本文件
```

---

## PC端 (Ubuntu 22.04)

### WiFi连接脚本

```bash
# 赋予执行权限
chmod +x ubuntu_wifi_connect.sh

# 交互式运行
sudo ./ubuntu_wifi_connect.sh

# 命令行模式
sudo ./ubuntu_wifi_connect.sh 1      # 连接i-HDU
sudo ./ubuntu_wifi_connect.sh 2      # 连接Undefined
sudo ./ubuntu_wifi_connect.sh --scan     # 扫描网络
sudo ./ubuntu_wifi_connect.sh --status   # 查看当前IP
sudo ./ubuntu_wifi_connect.sh --disconnect  # 断开连接
```

**WiFi配置（修改脚本顶部）：**
```bash
WIFI_1_NAME="i-HDU"
WIFI_1_PASSWORD=""
WIFI_1_BSSID="28:41:EC:29:30:10"

WIFI_2_NAME="Undefined"
WIFI_2_PASSWORD="lzh200341.."
WIFI_2_BSSID="36:C0:AA:ED:FC:62"
```

### 运行地面站

```bash
# 确保香橙派已连接WiFi并获取IP地址
# 修改ground_station.py中的IP:
# ORANGE_PI_IP = '香橙派IP地址'

# 运行地面站
python3 ground_station.py
```

---

## 香橙派 (OrangePi OS)

### WiFi连接脚本

**上传脚本到香橙派：**
```bash
scp wifi_connect.sh root@<香橙派IP>:/root/
```

**SSH连接香橙派：**
```bash
ssh root@<香橙派IP>
```

**在香橙派上操作：**
```bash
# 赋予执行权限
chmod +x wifi_connect.sh

# 交互式运行
sudo ./wifi_connect.sh

# 命令行模式
sudo ./wifi_connect.sh connect 1     # 连接i-HDU
sudo ./wifi_connect.sh connect 2     # 连接Undefined
sudo ./wifi_connect.sh --scan        # 扫描网络
sudo ./wifi_connect.sh --status      # 查看当前IP
sudo ./wifi_connect.sh --disconnect  # 断开连接

# 设置开机自启（连接i-HDU）
sudo ./wifi_connect.sh --autostart

# 取消开机自启
sudo ./wifi_connect.sh --remove-autostart
```

**WiFi配置（修改脚本顶部）：**
```bash
WIFI_1_NAME="i-HDU"
WIFI_1_PASSWORD=""
WIFI_1_BSSID="28:41:EC:29:30:10"

WIFI_2_NAME="Undefined"
WIFI_2_PASSWORD="lzh200341.."
WIFI_2_BSSID="36:C0:AA:ED:FC:62"
```

### 运行服务端

```bash
# 确保WiFi已连接，查看IP
./wifi_connect.sh --status

# 运行drone_server.py
python3 drone_server.py --pc-ip 192.168.1.50 --connection /dev/ttyACM0
```

**开机自启服务：**
```bash
# 设置开机自动连接i-HDU并运行服务端
./wifi_connect.sh --autostart
```

---

## i-HDU网络认证

连接i-HDU后，如需访问外网，需要网页认证：

1. 浏览器访问：`http://login.hdu.edu.cn`
2. 账号：`251050092`
3. 密码：`lzh0341..`

---

## 通信端口

| 端口 | 方向 | 说明 |
|-----|------|-----|
| TCP 5000 | PC→香橙派 | 控制指令 |
| TCP 5001 | 香橙派→PC | 状态回传 |
| UDP 14550 | MAVLink | 飞控通信 |

---

## 常见问题

### Q: WiFi连接失败？
A:
1. 确保目标WiFi在信号范围内
2. 检查WiFi密码是否正确
3. 尝试重新扫描：`sudo ./wifi_connect.sh --scan`
4. 检查BSSID是否正确

### Q: 无法连接到香橙派？
A:
1. 确保香橙派已连接WiFi并查看IP
2. 检查PC和香橙派在同一局域网
3. 检查防火墙设置

### Q: 地面站无响应？
A:
1. 确保drone_server.py正在香橙派上运行
2. 检查香橙派IP是否正确
3. 检查端口5000是否开放
