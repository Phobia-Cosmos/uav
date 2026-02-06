# Config (配置文件)

系统配置文件，包括飞控参数、网络配置等。

## 文件说明

| 文件 | 类型 | 描述 |
|-----|------|-----|
| `frame_config.parm` | 飞控参数 | 机身/框架配置参数 |
| `quad_config.parm` | 飞控参数 | 四旋翼无人机专用参数 |
| `quad_setup.parm` | 飞控参数 | 四旋翼初始设置参数 |
| `mav.parm` | 飞控参数 | MAVLink通信相关参数 |
| `network_config.json` | 网络配置 | 地空协同系统网络设置 |

## 配置文件格式

### *.parm 文件 (飞控参数)
```
参数名 = 值
例如：
ARMING_CHECK = 1
FS_THR_ENABLE = 1
```

### network_config.json
```json
{
    "wifi_profiles": {
        "2楼": {
            "pc": {"ip": "192.168.55.126", "ports": [5100, 5101]},
            "drone": {"ip": "192.168.55.128", "ports": [5200, 5300]},
            "dog": {"ip": "192.168.55.127", "port": 5400}
        }
    }
}
```

## 端口分配

| 设备 | 接收端口 | 用途 |
|-----|---------|------|
| PC (地面站) | 5100, 5101 | 接收无人机/机器狗状态 |
| 无人机 | 5200 | 接收PC命令 |
| 无人机 | 5300 | 与机器狗通信 |
| 机器狗 | 5400 | 接收无人机命令 |

## 使用方法

### 加载飞控参数
```bash
# 通过MAVLink加载参数
mavlink.py --load-setup config/quad_config.parm
```

### 网络配置
```bash
# 编辑配置
vim config/network_config.json

# 运行自动检测
python3 -c "from common import ConfigManager; c = ConfigManager(); c.detect_wifi()"
```

## 注意事项

1. 修改`.parm`文件前请备份
2. 网络配置需要与实际IP匹配
3. 端口冲突会导致通信失败
4. 修改飞控参数后需要重启飞控
