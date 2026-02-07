# 阶段四：实际环境适配

## 1. 概述

本阶段目标：
- 在真实环境中部署路径规划系统
- 集成 SLAM 和定位系统
- 实现实时障碍物检测
- 处理实际部署中的各种异常情况

## 2. 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                      实际部署架构                                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │                      机器狗系统                             │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │ │
│  │  │  传感器层    │→ │  感知层     │→ │  规划层     │     │ │
│  │  │  · 激光雷达  │  │  · SLAM    │  │  · A*      │     │ │
│  │  │  · 深度相机  │  │  · 障碍检测 │  │  · DWA     │     │ │
│  │  │  · IMU      │  │  · 定位     │  │  · 重规划   │     │ │
│  │  └─────────────┘  └─────────────┘  └─────────────┘     │ │
│  │         │                  │              │           │ │
│  │         ↓                  ↓              ↓           │ │
│  │  ┌───────────────────────────────────────────────────┐ │ │
│  │  │               执行层 / 通信层                      │ │ │
│  │  └───────────────────────────────────────────────────┘ │ │
│  └───────────────────────────────────────────────────────────┘ │
│                                                                   │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │                      无人机系统                           │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │ │
│  │  │  传感器层    │→ │  感知层     │→ │  规划层     │     │ │
│  │  │  · 激光雷达  │  │  · SLAM 3D │  │  · 3D A*   │     │ │
│  │  │  · 深度相机  │  │  · 视觉里程 │  │  · RRT     │     │ │
│  │  │  · GPS/RTK  │  │  · 障碍检测 │  │  · 重规划   │     │ │
│  │  └─────────────┘  └─────────────┘  └─────────────┘     │ │
│  │         │                  │              │           │ │
│  │         ↓                  ↓              ↓           │ │
│  │  ┌───────────────────────────────────────────────────┐ │ │
│  │  │               执行层 / 通信层                      │ │ │
│  │  └───────────────────────────────────────────────────┘ │ │
│  └───────────────────────────────────────────────────────────┘ │
│                                                                   │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │                      地面站系统 (PC)                       │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │ │
│  │  │  通信管理    │→ │  任务规划   │→ │  可视化界面  │     │ │
│  │  │  · TCP桥接   │  │  · 目标分配  │  │  · 状态监控  │     │ │
│  │  │  · 消息路由   │  │  · 协调决策  │  │  · 指令输入  │     │ │
│  │  └─────────────┘  └─────────────┘  └─────────────┘     │ │
│  └───────────────────────────────────────────────────────────┘ │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

## 3. 硬件需求

### 3.1 机器狗硬件配置

| 组件 | 型号/规格 | 说明 |
|------|----------|------|
| 主控 | Raspberry Pi 4B / Jetson Nano | 运行规划算法 |
| 激光雷达 | SLAMTEC RPLIDAR A1 | 360° 2D 扫描 |
| 深度相机 | Intel RealSense D435i | RGB-D 视觉 |
| IMU | MPU9250 | 姿态测量 |
| 通信 | WiFi 5GHz | 高速数据传输 |

### 3.2 无人机硬件配置

| 组件 | 型号/规格 | 说明 |
|------|----------|------|
| 主控 | NVIDIA Jetson Xavier NX | 运行 3D 规划 |
| 激光雷达 | Livox MID-40 | 3D 点云 |
| 深度相机 | Intel RealSense D455 | RGB-D 视觉 |
| 定位 | RTK GPS + 光流 | 高精度定位 |
| 飞控 | Pixhawk 4 | 底层控制 |
| 通信 | WiFi 5GHz | 数据传输 |

### 3.3 地面站配置

| 组件 | 要求 |
|------|------|
| CPU | Intel i5 以上 |
| 内存 | 8GB 以上 |
| 显卡 | 支持 OpenGL |
| 通信 | 双 WiFi (2.4G/5G) |

## 4. 软件架构

### 4.1 操作系统

```
机器狗: Ubuntu 20.04 LTS + ROS Noetic
无人机: Ubuntu 20.04 LTS + ROS Noetic + NVIDIA JetPack
PC: Ubuntu 20.04 LTS / Windows 10 + Python 3.8+
```

### 4.2 核心软件组件

```
src/
├── dog_system/
│   ├── dog_slam/           # SLAM 定位
│   │   ├── slam_node.py
│   │   └── map_builder.py
│   ├── dog_perception/     # 感知层
│   │   ├── obstacle_detector.py
│   │   └── point_cloud_processor.py
│   ├── dog_planning/       # 规划层
│   │   ├── a_star_2d.py
│   │   └── local_planner.py
│   ├── dog_control/        # 控制层
│   │   ├── motion_controller.py
│   │   └── gait_generator.py
│   └── dog_comm/          # 通信层
│       ├── tcp_client.py
│       └── message_bridge.py
│
├── uav_system/
│   ├── uav_slam/           # 3D SLAM
│   │   ├── visual_odometry.py
│   │   └── map_3d_builder.py
│   ├── uav_perception/     # 感知层
│   │   ├── point_cloud_3d.py
│   │   └── obstacle_3d.py
│   ├── uav_planning/       # 规划层
│   │   ├── a_star_3d.py
│   │   └── trajectory_generator.py
│   ├── uav_control/        # 控制层
│   │   ├── attitude_controller.py
│   │   └── trajectory_tracker.py
│   └── uav_comm/          # 通信层
│       ├── tcp_server.py
│       └── mavlink_bridge.py
│
└── ground_station/
    ├── gcs_interface.py    # 地面站界面
    ├── mission_planner.py   # 任务规划
    └── visualizer_3d.py    # 3D 可视化
```

## 5. SLAM 与定位系统

### 5.1 机器狗 SLAM

```python
# dog_slam/slam_node.py

import rospy
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, Twist
import numpy as np


class DogSLAMNode:
    """机器狗 SLAM 节点"""

    def __init__(self):
        rospy.init_node('dog_slam', anonymous=True)

        # 订阅话题
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)

        # 发布话题
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

        # SLAM 状态
        self.pose = (0, 0, 0)  # x, y, theta
        self.map_data = None
        self.resolution = 0.05  # 5cm per pixel
        self.map_size = (100, 100)  # 5m x 5m

    def scan_callback(self, scan_msg: LaserScan):
        """处理激光雷达数据"""
        # 更新地图
        self._update_map(scan_msg)

        # 更新位姿
        self._update_pose(scan_msg)

        # 发布里程计
        self._publish_odom()

    def imu_callback(self, imu_msg: Imu):
        """处理 IMU 数据"""
        # 融合 IMU 数据提高精度
        pass

    def _update_map(self, scan_msg: LaserScan):
        """更新占据地图"""
        pass

    def _update_pose(self, scan_msg: LaserScan):
        """更新位姿估计"""
        pass

    def _publish_odom(self):
        """发布里程计"""
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position.x = self.pose[0]
        odom.pose.pose.position.y = self.pose[1]
        odom.pose.pose.orientation.z = self.pose[2]
        self.odom_pub.publish(odom)

    def run(self):
        """运行 SLAM 节点"""
        rospy.spin()


if __name__ == '__main__':
    slam = DogSLAMNode()
    slam.run()
```

### 5.2 无人机 3D SLAM

```python
# uav_slam/visual_odometry.py

import rospy
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np


class UAVVisualOdometry:
    """无人机视觉里程计"""

    def __init__(self):
        rospy.init_node('uav_visual_odometry', anonymous=True)

        # 相机内参 (需要标定)
        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])

        # 当前位姿
        self.pose = np.eye(4)  # 4x4 变换矩阵

        # 特征跟踪
        self.prev_features = None
        self.prev_frame = None

    def process_frame(self, rgb_msg: Image, depth_msg: Image):
        """处理一帧图像"""
        # 转换为 OpenCV 格式
        rgb = self._image_to_numpy(rgb_msg)
        depth = self._image_to_numpy(depth_msg)

        # 特征提取
        features = self._extract_features(rgb)

        # 特征匹配与运动估计
        if self.prev_features is not None:
            matches = self._match_features(self.prev_features, features)
            self._estimate_motion(matches, rgb, depth)

        # 更新状态
        self.prev_features = features
        self.prev_frame = rgb

    def _extract_features(self, frame):
        """提取特征点"""
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # 使用 ORB 特征
        orb = cv2.ORB_create()
        keypoints = orb.detect(gray, None)
        keypoints, descriptors = orb.compute(gray, keypoints)
        return {'kp': keypoints, 'desc': descriptors, 'frame': gray}

    def _match_features(self, prev, curr):
        """特征匹配"""
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(prev['desc'], curr['desc'])
        return matches

    def _estimate_motion(self, matches, rgb, depth):
        """估计运动"""
        pass

    def _image_to_numpy(self, ros_image):
        """ROS Image 转 numpy"""
        pass


if __name__ == '__main__':
    vo = UAVVisualOdometry()
```

## 6. 障碍物检测与处理

### 6.1 2D 障碍物检测 (机器狗)

```python
# dog_perception/obstacle_detector.py

import numpy as np
from typing import List, Tuple, Dict


class ObstacleDetector2D:
    """2D 障碍物检测器"""

    def __init__(self, safety_margin: float = 0.5):
        self.safety_margin = safety_margin
        self.resolution = 0.05  # 5cm

    def detect_from_scan(self, scan_data: List[float],
                         angles: List[float]) -> List[Tuple[float, float, float]]:
        """
        从激光雷达数据中检测障碍物

        Returns:
            List of (x, y, radius) obstacles
        """
        obstacles = []

        # 过滤无效数据
        valid_ranges = []
        valid_angles = []
        for r, theta in zip(scan_data, angles):
            if 0 < r < 10:  # 有效范围
                valid_ranges.append(r)
                valid_angles.append(theta)

        # 聚类分析
        if len(valid_ranges) > 0:
            obstacles = self._cluster_obstacles(valid_ranges, valid_angles)

        return obstacles

    def _cluster_obstacles(self, ranges: List[float],
                           angles: List[float]) -> List[Tuple[float, float, float]]:
        """聚类检测到的点"""
        # 将极坐标转换为笛卡尔坐标
        points = [(r * np.cos(theta), r * np.sin(theta))
                  for r, theta in zip(ranges, angles)]

        # 简化的聚类 (实际应该使用 DBSCAN)
        clusters = []
        visited = set()

        for i, point in enumerate(points):
            if i in visited:
                continue

            cluster = [point]
            visited.add(i)

            for j, other in enumerate(points):
                if j in visited:
                    continue

                dist = np.sqrt((point[0] - other[0])**2 +
                              (point[1] - other[1])**2)

                if dist < 0.3:  # 聚类阈值
                    cluster.append(other)
                    visited.add(j)

            if len(cluster) > 3:  # 至少3个点
                # 计算障碍物中心
                cx = np.mean([p[0] for p in cluster])
                cy = np.mean([p[1] for p in cluster])
                # 计算近似半径
                radius = max([np.sqrt((p[0] - cx)**2 + (p[1] - cy)**2) for p in cluster])
                clusters.append((cx, cy, radius + self.safety_margin))

        return clusters

    def inflate_obstacles(self, obstacles: List[Tuple[float, float, float]],
                         inflation_radius: float) -> List[Tuple[float, float, float]]:
        """障碍物膨胀"""
        inflated = []
        for obs in obstacles:
            cx, cy, r = obs
            inflated.append((cx, cy, r + inflation_radius))
        return inflated
```

### 6.2 3D 障碍物检测 (无人机)

```python
# uav_perception/obstacle_3d.py

import numpy as np
from typing import List, Tuple
from dataclasses import dataclass


@dataclass
class Obstacle3D:
    """3D 障碍物"""
    center: Tuple[float, float, float]
    size: Tuple[float, float, float]  # length, width, height


class ObstacleDetector3D:
    """3D 障碍物检测器"""

    def __init__(self, resolution: float = 0.1):
        self.resolution = resolution

    def detect_from_pointcloud(self, pointcloud: np.ndarray) -> List[Obstacle3D]:
        """
        从点云中检测3D障碍物

        Args:
            pointcloud: Nx3 点云 (x, y, z)

        Returns:
            List of Obstacle3D
        """
        obstacles = []

        # 过滤地面点
        ground_height = 0.1
        valid_points = pointcloud[pointcloud[:, 2] > ground_height]

        if len(valid_points) == 0:
            return obstacles

        # 简化的障碍物检测 (实际应使用 RANSAC 或欧几里得聚类)
        # 按高度分层
        height_bins = np.arange(ground_height, 5.0, 0.5)

        for i in range(len(height_bins) - 1):
            layer_points = valid_points[
                (valid_points[:, 2] >= height_bins[i]) &
                (valid_points[:, 2] < height_bins[i + 1])
            ]

            if len(layer_points) < 10:
                continue

            # 计算边界框
            min_pt = layer_points.min(axis=0)
            max_pt = layer_points.max(axis=0)

            center = ((min_pt + max_pt) / 2).tolist()
            size = (max_pt - min_pt).tolist()

            obstacles.append(Obstacle3D(
                center=(center[0], center[1], center[2]),
                size=(size[0], size[1], size[2])
            ))

        return obstacles

    def generate_voxel_map(self, pointcloud: np.ndarray,
                          voxel_size: float = 0.1) -> np.ndarray:
        """生成体素地图"""
        pass
```

## 7. 实际部署步骤

### 7.1 环境准备

```bash
# 1. 安装 Ubuntu 20.04
# 下载地址: https://releases.ubuntu.com/20.04/

# 2. 安装 ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt update
sudo apt install ros-noetic-desktop-full

# 3. 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash

# 4. 安装依赖
sudo apt install python3-pip python3-numpy python3-opencv
pip3 install scipy numpy matplotlib
```

### 7.2 设备配置

#### 机器狗配置

```bash
# SSH 连接
ssh orangepi@192.168.55.127

# 配置 WiFi
sudo nmcli device wifi list
sudo nmcli device wifi connect "2楼" password "your_password"

# 设置静态 IP
sudo nmcli connection modify "2楼" ipv4.addresses 192.168.55.127/24
sudo nmcli connection modify "2楼" ipv4.method manual
sudo systemctl restart NetworkManager
```

#### 无人机配置

```bash
# SSH 连接
ssh orangepi@192.168.55.128

# 配置 WiFi (同上)

# 配置 RTK GPS
# 1. 连接 RTK 基站
# 2. 配置 NTRIP 客户端
# 3. 设置 RTK 模式为 Fixed
```

### 7.3 启动脚本

#### 机器狗启动脚本

```bash
#!/bin/bash
# launch_dog.sh

source ~/catkin_ws/devel/setup.bash

# 启动 SLAM
roslaunch dog_slam rplidar_slam.launch &

# 启动障碍物检测
roslaunch dog_perception obstacle_detector.launch &

# 启动路径规划
roslaunch dog_planning path_planner.launch &

# 启动通信客户端
roslaunch dog_comm tcp_client.launch &

echo "Dog system started"
```

#### 无人机启动脚本

```bash
#!/bin/bash
# launch_uav.sh

source ~/catkin_ws/devel/setup.bash

# 启动飞控连接
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600 &

# 启动 SLAM
roslaunch uav_slam visual_slam.launch &

# 启动 3D 障碍物检测
roslaunch uav_perception obstacle_3d.launch &

# 启动 3D 路径规划
roslaunch uav_planning planner_3d.launch &

# 启动通信服务端
roslaunch uav_comm tcp_server.launch &

echo "UAV system started"
```

### 7.4 测试流程

```bash
# 1. 机器狗测试
ssh orangepi@192.168.55.127
./launch_dog.sh

# 检查话题
rostopic list
rostopic echo /odom
rostopic echo /scan

# 2. 无人机测试
ssh orangepi@192.168.55.128
./launch_uav.sh

# 检查话题
rostopic list
rostopic echo /mavros/local_position/pose

# 3. 地面站测试
cd ~/ground_station
python3 main.py
```

## 8. 异常处理

### 8.1 常见问题与解决方案

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| SLAM 丢失 | 快速运动/纹理缺失 | 降低速度/添加视觉标记 |
| GPS 信号弱 | 室内/遮挡 | 切换到视觉里程计 |
| 激光雷达数据异常 | 镜面反射/强光 | 过滤异常值 |
| 通信延迟高 | WiFi 干扰 | 使用 5GHz 频段/降低数据量 |
| 飞控断开 | USB 连接不稳 | 检查线缆/增加供电 |

### 8.2 安全机制

```python
# safety_monitor.py

class SafetyMonitor:
    """安全监控器"""

    def __init__(self, device_type: str):
        self.device_type = device_type
        self.emergency_stop = False
        self.low_battery = False
        self.sensor_failure = False

    def check_battery(self, voltage: float, threshold: float = 11.1) -> bool:
        """检查电池电压"""
        if voltage < threshold:
            self.low_battery = True
            self._trigger_emergency("Low battery!")
            return True
        return False

    def check_connection(self, heartbeat_lost: bool) -> None:
        """检查通信连接"""
        if heartbeat_lost:
            self._trigger_emergency("Heartbeat lost!")

    def check_obstacles(self, distance: float, min_safe: float = 0.5) -> bool:
        """检查障碍物距离"""
        if distance < min_safe:
            self._trigger_emergency(f"Obstacle too close: {distance}m")
            return True
        return False

    def _trigger_emergency(self, reason: str):
        """触发紧急停止"""
        print(f"[SAFETY] Emergency stop: {reason}")
        self.emergency_stop = True

    def reset(self):
        """重置安全状态"""
        self.emergency_stop = False
        self.low_battery = False
        self.sensor_failure = False
```

## 9. 性能指标

### 9.1 规划性能要求

| 指标 | 机器狗 | 无人机 |
|------|--------|--------|
| 规划频率 | ≥ 1 Hz | ≥ 2 Hz |
| 规划时间 | < 500ms | < 200ms |
| 路径质量 | 短路径、低转向 | 3D 平滑 |
| 重规划时间 | < 1s | < 500ms |

### 9.2 定位精度要求

| 场景 | 水平精度 | 垂直精度 |
|------|----------|----------|
| 室外 (RTK) | < 5cm | < 10cm |
| 室外 (GPS) | < 1m | < 2m |
| 室内 (SLAM) | < 10cm | < 5cm |

### 9.3 通信性能要求

| 指标 | 要求 |
|------|------|
| 延迟 | < 100ms |
| 丢包率 | < 1% |
| 带宽 | ≥ 10 Mbps |

## 10. 后续优化方向

1. **算法优化**
   - 实现动态窗口 DWA 算法
   - 集成 Learning-based 路径预测
   - 多智能体协同规划

2. **硬件升级**
   - 更高精度激光雷达
   - 边缘计算设备 (NVIDIA Jetson AGX)
   - 冗余传感器配置

3. **系统可靠性**
   - 实现故障转移机制
   - 添加看门狗监控
   - 实现安全模式降落
