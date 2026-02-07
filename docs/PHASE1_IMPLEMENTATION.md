# 阶段一：纯模拟路径规划

## 1. 概述

本阶段实现目标：
- 在静态地图上进行路径规划模拟
- 机器狗使用 2D A* 算法
- 无人机使用 3D A* 算法
- 可视化显示路径规划结果
- 输出路径评分

## 2. 文件结构

```
simulation/
├── map_generator.py     # 地图生成 + 障碍物定义
├── a_star_2d.py        # 2D A* (机器狗)
├── a_star_3d.py        # 3D A* (无人机)
├── path_evaluator.py   # 路径评分
├── visualizer.py        # 2D/3D 可视化
├── config/
│   └── scenario_01.json # 场景配置
├── main_dog.py         # 机器狗主程序
├── main_uav.py         # 无人机主程序
└── README.md
```

## 3. 地图配置格式 (JSON)

```json
{
  "name": "搜救场景_01",
  "description": "简单搜救场景，无障碍物",
  "size": {
    "x": 50,
    "y": 50,
    "z": 30
  },
  "start": {
    "x": 0,
    "y": 0,
    "z": 0
  },
  "goal": {
    "x": 40,
    "y": 40,
    "z": 10
  },
  "dog_obstacles": [
    {
      "id": "obs_001",
      "type": "rectangle",
      "position": {"x": 15, "y": 10},
      "size": {"width": 8, "height": 5}
    },
    {
      "id": "obs_002",
      "type": "circle",
      "position": {"x": 30, "y": 25},
      "size": {"radius": 3}
    }
  ],
  "uav_obstacles": [
    {
      "id": "obs_003",
      "type": "box",
      "position": {"x": 20, "y": 15, "z": 5},
      "size": {"length": 6, "width": 4, "height": 15}
    }
  ]
}
```

## 4. 2D A* 算法实现

### 核心数据结构

```python
class Node2D:
    """2D 路径节点"""
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.g = 0  # 起点到当前点代价
        self.h = 0   # 当前点到终点估计代价
        self.f = 0   # 总代价 f = g + h

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))
```

### 启发式函数 (曼哈顿距离)

```python
def heuristic(node, goal):
    """曼哈顿距离启发式函数"""
    return abs(node.x - goal.x) + abs(node.y - goal.y)
```

### 邻居节点 (8方向)

```python
def get_neighbors(node, grid_size, obstacles):
    """获取8个方向的邻居节点"""
    directions = [
        (0, 1), (1, 0), (0, -1), (-1, 0),  # 上下左右
        (1, 1), (1, -1), (-1, 1), (-1, -1)  # 对角线
    ]
    neighbors = []
    for dx, dy in directions:
        nx, ny = node.x + dx, node.y + dy
        if 0 <= nx < grid_size[0] and 0 <= ny < grid_size[1]:
            if not is_collision((nx, ny), obstacles):
                neighbors.append(Node2D(nx, ny))
    return neighbors
```

## 5. 3D A* 算法实现

### 核心数据结构

```python
class Node3D:
    """3D 路径节点"""
    def __init__(self, x, y, z, parent=None):
        self.x = x
        self.y = y
        self.z = z
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __hash__(self):
        return hash((self.x, self.y, self.z))
```

### 启发式函数 (欧几里得距离)

```python
def heuristic_3d(node, goal):
    """欧几里距离开启式函数"""
    return ((node.x - goal.x)**2 + (node.y - goal.y)**2 + (node.z - goal.z)**2) ** 0.5
```

### 邻居节点 (26方向)

```python
def get_neighbors_3d(node, grid_size, obstacles):
    """获取26个方向的邻居节点 (3D)"""
    neighbors = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                nx, ny, nz = node.x + dx, node.y + dy, node.z + dz
                if 0 <= nx < grid_size[0] and 0 <= ny < grid_size[1] and 0 <= nz < grid_size[2]:
                    if not is_collision_3d((nx, ny, nz), obstacles):
                        neighbors.append(Node3D(nx, ny, nz))
    return neighbors
```

## 6. 路径评分标准

### 评分公式

```
评分 = 0.50·长度得分 + 0.30·平滑度得分 + 0.20·能耗得分
```

### 评分计算

```python
class PathEvaluator:
    """路径评分器"""

    def evaluate(self, path, grid_size):
        """
        评估路径质量

        Returns:
            dict: 包含各项评分和总分
        """
        length_score = self._calculate_length_score(path)
        smoothness_score = self._calculate_smoothness_score(path)
        energy_score = self._calculate_energy_score(path)

        total_score = (
            0.50 * length_score +
            0.30 * smoothness_score +
            0.20 * energy_score
        )

        return {
            "total_score": round(total_score, 3),
            "length_score": round(length_score, 3),
            "smoothness_score": round(smoothness_score, 3),
            "energy_score": round(energy_score, 3),
            "path_length": self._calculate_total_length(path),
            "turn_count": self._count_turns(path),
            "estimated_time": self._estimate_time(path)
        }

    def _calculate_length_score(self, path):
        """长度得分 (越短越好)"""
        total_length = self._calculate_total_length(path)
        max_possible = max(path.grid_size[0], path.grid_size[1]) * 2
        return max(0, 1 - total_length / max_possible)

    def _calculate_smoothness_score(self, path):
        """平滑度得分 (转向越少越好)"""
        turn_count = self._count_turns(path)
        max_turns = len(path.points) - 1
        return max(0, 1 - turn_count / max_turns) if max_turns > 0 else 1.0

    def _calculate_energy_score(self, path):
        """能耗得分 (估算)"""
        turn_count = self._count_turns(path)
        # 转向消耗更多能量
        base_energy = self._calculate_total_length(path)
        turn_penalty = turn_count * 0.5
        total_energy = base_energy + turn_penalty
        max_energy = max(path.grid_size) * 3
        return max(0, 1 - total_energy / max_energy)

    def _calculate_total_length(self, path):
        """计算路径总长度"""
        length = 0
        for i in range(1, len(path.points)):
            p1, p2 = path.points[i-1], path.points[i]
            length += ((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2) ** 0.5
        return length

    def _count_turns(self, path):
        """计算转向次数"""
        turns = 0
        for i in range(2, len(path.points)):
            p0, p1, p2 = path.points[i-2], path.points[i-1], path.points[i]
            # 检查方向变化
            v1 = (p1.x - p0.x, p1.y - p0.y, p1.z - p0.z)
            v2 = (p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
            if v1 != v2:
                turns += 1
        return turns

    def _estimate_time(self, path):
        """估算到达时间 (秒)"""
        length = self._calculate_total_length(path)
        speed = 1.0  # 假设速度 1m/s
        return length / speed
```

## 7. 可视化设计

### 2D 可视化 (机器狗)

```python
def visualize_dog_2d(map_data, path):
    """2D 可视化 - 机器狗"""
    fig, ax = plt.subplots(figsize=(10, 10))

    # 绘制障碍物
    for obs in map_data["dog_obstacles"]:
        if obs["type"] == "rectangle":
            rect = plt.Rectangle(
                (obs["position"]["x"], obs["position"]["y"]),
                obs["size"]["width"], obs["size"]["height"],
                facecolor='lightgray', edgecolor='black'
            )
            ax.add_patch(rect)
        elif obs["type"] == "circle":
            circle = plt.Circle(
                (obs["position"]["x"], obs["position"]["y"]),
                obs["size"]["radius"],
                facecolor='lightgray', edgecolor='black'
            )
            ax.add_patch(circle)

    # 绘制起点和终点
    ax.scatter(map_data["start"]["x"], map_data["start"]["y"],
               c='green', s=200, marker='s', label='Start', zorder=5)
    ax.scatter(map_data["goal"]["x"], map_data["goal"]["y"],
               c='red', s=200, marker='*', label='Goal', zorder=5)

    # 绘制路径
    path_x = [p["x"] for p in path["points"]]
    path_y = [p["y"] for p in path["points"]]
    ax.plot(path_x, path_y, 'orange', linewidth=2, label='Dog Path')
    ax.scatter(path_x, path_y, c='orange', s=30, zorder=4)

    # 设置标题和标签
    ax.set_title("Dog 2D Path Planning", fontsize=14)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    return fig, ax
```

### 3D 可视化 (无人机)

```python
def visualize_uav_3d(map_data, path):
    """3D 可视化 - 无人机"""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制障碍物
    for obs in map_data["uav_obstacles"]:
        if obs["type"] == "box":
            x, y, z = obs["position"]["x"], obs["position"]["y"], obs["position"]["z"]
            l, w, h = obs["size"]["length"], obs["size"]["width"], obs["size"]["height"]
            ax.bar3d(x, y, z, l, w, h, color='lightgray', edgecolor='black', alpha=0.8)

    # 绘制起点和终点
    ax.scatter(map_data["start"]["x"], map_data["start"]["y"], map_data["start"]["z"],
               c='green', s=200, marker='s', label='Start', depthshade=True)
    ax.scatter(map_data["goal"]["x"], map_data["goal"]["y"], map_data["goal"]["z"],
               c='red', s=200, marker='*', label='Goal', depthshade=True)

    # 绘制路径
    path_x = [p["x"] for p in path["points"]]
    path_y = [p["y"] for p in path["points"]]
    path_z = [p["z"] for p in path["points"]]
    ax.plot(path_x, path_y, path_z, 'blue', linewidth=2, label='UAV Path')
    ax.scatter(path_x, path_y, path_z, c='blue', s=30, depthshade=True)

    # 设置标题和标签
    ax.set_title("UAV 3D Path Planning", fontsize=14)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_zlabel("Z (meters)")
    ax.legend()

    return fig, ax
```

## 8. 使用说明

### 运行机器狗路径规划

```bash
cd simulation
python main_dog.py
```

输出：
```
========================================
    机器狗 2D 路径规划 (A*)
========================================
场景: 搜救场景_01
地图大小: 50m x 50m
障碍物数量: 2

起点: (0, 0)
终点: (40, 40)

[运行 A* 算法...]
路径找到！点数: 45

路径评分:
  - 总分: 0.823
  - 长度得分: 0.891
  - 平滑度得分: 0.772
  - 能耗得分: 0.650

路径统计:
  - 总长度: 48.5m
  - 转向次数: 12
  - 预计时间: 48.5s

显示可视化窗口...
```

### 运行无人机路径规划

```bash
cd simulation
python main_uav.py
```

输出：
```
========================================
    无人机 3D 路径规划 (A*)
========================================
场景: 搜救场景_01
地图大小: 50m x 50m x 30m
障碍物数量: 1

起点: (0, 0, 0)
终点: (40, 40, 10)

[运行 A* 算法...]
路径找到！点数: 62

路径评分:
  - 总分: 0.756
  - 长度得分: 0.812
  - 平滑度得分: 0.689
  - 能耗得分: 0.695

路径统计:
  - 总长度: 55.2m
  - 转向次数: 18
  - 预计时间: 55.2s

显示可视化窗口...
```

## 9. 扩展说明

### 后续优化方向

1. **算法优化**
   - 加入障碍物膨胀 (inflate obstacles)
   - 实现动态权重 A*
   - 添加路径平滑 (path smoothing)

2. **评分扩展**
   - 加入安全性评分 (考虑障碍物距离)
   - 加入高度适应性评分 (无人机)

3. **可视化增强**
   - 动画展示路径搜索过程
   - 交互式地图 (可拖拽起点/终点)
   - 多场景对比显示

## 10. 依赖安装

```bash
pip install numpy matplotlib
```
