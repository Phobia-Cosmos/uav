#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D A* Path Planning for UAV

无人机 3D 路径规划算法实现。
"""

import heapq
import math
from typing import List, Tuple, Dict, Optional, Set
from dataclasses import dataclass, field
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from map_generator import Obstacle3D, load as load_map


@dataclass
class Node3D:
    """3D 路径节点"""
    x: int
    y: int
    z: int
    g: float = field(default=0, repr=False)
    h: float = field(default=0, repr=False)
    f: float = field(default=0, repr=False)
    parent: Optional['Node3D'] = field(default=None, repr=False)

    def __hash__(self):
        return hash((self.x, self.y, self.z))

    def __eq__(self, other):
        return (self.x == other.x and
                self.y == other.y and
                self.z == other.z)

    def __lt__(self, other):
        return self.f < other.f


class AStar3D:
    """3D A* 路径规划器"""

    def __init__(self, obstacles: List[Dict],
                 grid_size: Tuple[int, int, int] = (50, 50, 30),
                 resolution: float = 1.0):
        """
        初始化 A* 规划器

        Args:
            obstacles: 障碍物列表 (字典格式)
            grid_size: 网格尺寸 (x, y, z)
            resolution: 分辨率 (每个网格代表多少米)
        """
        self.width = grid_size[0]
        self.height = grid_size[1]
        self.depth = grid_size[2]
        self.resolution = resolution
        self.obstacles = [Obstacle3D.from_dict(obs) for obs in obstacles]

    def heuristic(self, node: Tuple[int, int, int],
                 goal: Tuple[int, int, int]) -> float:
        """
        启发式函数 (欧几里得距离)

        Args:
            node: 当前节点
            goal: 目标节点

        Returns:
            估计代价
        """
        return math.sqrt(
            (node[0] - goal[0])**2 +
            (node[1] - goal[1])**2 +
            (node[2] - goal[2])**2
        )

    def is_collision(self, point: Tuple[int, int, int]) -> bool:
        """检查点是否与障碍物碰撞"""
        x, y, z = point[0] * self.resolution, point[1] * self.resolution, point[2] * self.resolution

        for obs in self.obstacles:
            if obs.contains((x, y, z)):
                return True
        return False

    def get_neighbors(self, node: Node3D) -> List[Node3D]:
        """
        获取邻居节点 (26方向)

        Args:
            node: 当前节点

        Returns:
            邻居节点列表
        """
        neighbors = []

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue

                    nx, ny, nz = node.x + dx, node.y + dy, node.z + dz

                    # 检查边界
                    if (0 <= nx < self.width and
                        0 <= ny < self.height and
                        0 <= nz < self.depth):
                        if not self.is_collision((nx, ny, nz)):
                            neighbors.append(Node3D(x=nx, y=ny, z=nz, g=float('inf')))

        return neighbors

    def calculate_cost(self, from_node: Node3D,
                        to_node: Node3D) -> float:
        """
        计算移动代价

        Args:
            from_node: 起始节点
            to_node: 目标节点

        Returns:
            移动代价
        """
        dx = abs(to_node.x - from_node.x)
        dy = abs(to_node.y - from_node.y)
        dz = abs(to_node.z - from_node.z)

        moves = sum([dx, dy, dz])

        if moves == 1:
            return 1.0  # 面
        elif moves == 2:
            return math.sqrt(2)  # 边
        else:
            return math.sqrt(3)  # 角

    def plan(self, start: Tuple[float, float, float],
             goal: Tuple[float, float, float],
             inflate_radius: int = 0) -> Optional[Dict]:
        """
        执行 A* 3D 路径规划

        Args:
            start: 起点坐标 (x, y, z)
            goal: 终点坐标 (x, y, z)
            inflate_radius: 障碍物膨胀半径 (网格数)

        Returns:
            路径数据字典，或 None (如果无法规划)
        """
        # 转换为网格坐标
        start_node = Node3D(
            x=int(start[0] / self.resolution),
            y=int(start[1] / self.resolution),
            z=int(start[2] / self.resolution)
        )
        goal_node = Node3D(
            x=int(goal[0] / self.resolution),
            y=int(goal[1] / self.resolution),
            z=int(goal[2] / self.resolution)
        )

        # 检查起点和终点是否有效
        if self.is_collision((start_node.x, start_node.y, start_node.z)):
            print(f"[A* 3D] 起点与障碍物碰撞!")
            return None

        if self.is_collision((goal_node.x, goal_node.y, goal_node.z)):
            print(f"[A* 3D] 终点与障碍物碰撞!")
            return None

        # 膨胀障碍物
        inflated_obstacles = self._inflate_obstacles(inflate_radius)

        # A* 算法
        open_set = []
        closed_set: Set[Tuple[int, int, int]] = set()

        start_node.g = 0
        start_node.h = self.heuristic(
            (start_node.x, start_node.y, start_node.z),
            (goal_node.x, goal_node.y, goal_node.z)
        )
        start_node.f = start_node.g + start_node.h
        heapq.heappush(open_set, start_node)

        while open_set:
            current = heapq.heappop(open_set)

            # 到达目标
            if (current.x == goal_node.x and
                current.y == goal_node.y and
                current.z == goal_node.z):
                path = self._reconstruct_path(current)
                return {
                    "points": [{"x": p[0], "y": p[1], "z": p[2]} for p in path],
                    "start": {"x": start[0], "y": start[1], "z": start[2]},
                    "goal": {"x": goal[0], "y": goal[1], "z": goal[2]},
                    "algorithm": "A* (Euclidean)",
                    "grid_size": self.width,
                    "inflate_radius": inflate_radius
                }

            # 跳过已访问节点
            if (current.x, current.y, current.z) in closed_set:
                continue

            closed_set.add((current.x, current.y, current.z))

            # 扩展邻居
            for neighbor in self.get_neighbors(current):
                if (neighbor.x, neighbor.y, neighbor.z) in closed_set:
                    continue

                # 临时膨胀检查
                if inflated_obstacles and self._is_in_inflated(
                        neighbor.x, neighbor.y, neighbor.z, inflated_obstacles):
                    continue

                tentative_g = current.g + self.calculate_cost(current, neighbor)

                if tentative_g < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = tentative_g
                    neighbor.h = self.heuristic(
                        (neighbor.x, neighbor.y, neighbor.z),
                        (goal_node.x, goal_node.y, goal_node.z)
                    )
                    neighbor.f = neighbor.g + neighbor.h
                    heapq.heappush(open_set, neighbor)

        print(f"[A* 3D] 未找到路径!")
        return None

    def _inflate_obstacles(self, radius: int) -> Set[Tuple[int, int, int]]:
        """膨胀障碍物"""
        if radius <= 0:
            return set()

        inflated = set()

        for obs in self.obstacles:
            if obs.type == "box":
                # 计算膨胀后的边界
                l = obs.size["length"] / self.resolution + 2 * radius
                w = obs.size["width"] / self.resolution + 2 * radius
                h = obs.size["height"] / self.resolution + 2 * radius

                ox = int(obs.position.x / self.resolution)
                oy = int(obs.position.y / self.resolution)
                oz = int(obs.position.z / self.resolution)

                # 遍历膨胀区域
                for x in range(int(ox - l//2 - radius), int(ox + l//2 + radius + 1)):
                    for y in range(int(oy - w//2 - radius), int(oy + w//2 + radius + 1)):
                        for z in range(int(oz - h//2 - radius), int(oz + h//2 + radius + 1)):
                            if (0 <= x < self.width and
                                0 <= y < self.height and
                                0 <= z < self.depth):
                                # 检查是否在膨胀区域内
                                if ((x - ox)**2 / (l/2 + radius)**2 +
                                    (y - oy)**2 / (w/2 + radius)**2 +
                                    (z - oz)**2 / (h/2 + radius)**2 <= 1):
                                    inflated.add((x, y, z))

        return inflated

    def _is_in_inflated(self, x: int, y: int, z: int,
                         inflated: Set[Tuple[int, int, int]]) -> bool:
        """检查点是否在膨胀区域内"""
        return (x, y, z) in inflated

    def _reconstruct_path(self, end_node: Node3D) -> List[Tuple[float, float, float]]:
        """重建路径"""
        path = []
        current = end_node

        while current:
            path.append((
                current.x * self.resolution,
                current.y * self.resolution,
                current.z * self.resolution
            ))
            current = current.parent

        return list(reversed(path))


def main():
    """测试 A* 3D 路径规划"""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    # 加载地图
    config_path = os.path.join(os.path.dirname(__file__), "config/scenario_01.json")
    map_data = load_map(config_path)

    print("=" * 50)
    print("    无人机 3D 路径规划 (A*)")
    print("=" * 50)
    print(f"\n场景: {map_data['name']}")
    print(f"地图大小: {map_data['size']['x']}m x {map_data['size']['y']}m x {map_data['size']['z']}m")
    print(f"障碍物数量: {len(map_data['uav_obstacles'])}")

    # 提取起点和终点
    start = (map_data['start']['x'], map_data['start']['y'], map_data['start']['z'])
    goal = (map_data['goal']['x'], map_data['goal']['y'], map_data['goal']['z'])
    obstacles = map_data['uav_obstacles']

    print(f"\n起点: ({start[0]}, {start[1]}, {start[2]})")
    print(f"终点: ({goal[0]}, {goal[1]}, {goal[2]})")

    # 创建规划器
    grid_size = (
        map_data['size']['x'],
        map_data['size']['y'],
        map_data['size']['z']
    )
    planner = AStar3D(obstacles, grid_size)

    # 执行规划
    print(f"\n[运行 A* 算法...]")
    path_result = planner.plan(start, goal, inflate_radius=1)

    if path_result:
        print(f"路径找到! 点数: {len(path_result['points'])}")

        # 计算路径长度
        points = path_result['points']
        total_length = 0
        for i in range(1, len(points)):
            dx = points[i]['x'] - points[i-1]['x']
            dy = points[i]['y'] - points[i-1]['y']
            dz = points[i]['z'] - points[i-1]['z']
            total_length += math.sqrt(dx**2 + dy**2 + dz**2)

        print(f"路径总长度: {total_length:.2f}m")

        # 3D 可视化
        fig = plt.figure(figsize=(14, 6))

        # 3D 视图
        ax1 = fig.add_subplot(121, projection='3d')

        # 绘制障碍物
        for obs in obstacles:
            x = obs['position']['x']
            y = obs['position']['y']
            z = obs['position']['z']
            l = obs['size']['length']
            w = obs['size']['width']
            h = obs['size']['height']

            ax1.bar3d(x - l/2, y - w/2, z - h/2, l, w, h,
                      color='lightgray', edgecolor='black', alpha=0.8)

        # 绘制起点和终点
        ax1.scatter(start[0], start[1], start[2], c='green', s=200,
                    marker='s', label='Start', depthshade=True)
        ax1.scatter(goal[0], goal[1], goal[2], c='red', s=200,
                    marker='*', label='Goal', depthshade=True)

        # 绘制路径
        px = [p['x'] for p in path_result['points']]
        py = [p['y'] for p in path_result['points']]
        pz = [p['z'] for p in path_result['points']]
        ax1.plot(px, py, pz, 'blue', linewidth=2, label='UAV Path')
        ax1.scatter(px, py, pz, c='blue', s=30, depthshade=True)

        ax1.set_title(f"UAV 3D Path Planning (A*)\nLength: {total_length:.2f}m", fontsize=12)
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.set_zlabel("Z (m)")
        ax1.legend()

        # 2D 俯视图
        ax2 = fig.add_subplot(122)
        ax2.set_title("Top View", fontsize=12)

        for obs in obstacles:
            x = obs['position']['x']
            y = obs['position']['y']
            l = obs['size']['length']
            w = obs['size']['width']
            rect = plt.Rectangle(
                (x - l/2, y - w/2), l, w,
                facecolor='lightgray', edgecolor='black'
            )
            ax2.add_patch(rect)

        ax2.scatter(start[0], start[1], c='green', s=200, marker='s', label='Start')
        ax2.scatter(goal[0], goal[1], c='red', s=200, marker='*', label='Goal')
        ax2.plot(px, py, 'blue', linewidth=2, label='UAV Path')

        ax2.set_xlabel("X (m)")
        ax2.set_ylabel("Y (m)")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        ax2.set_aspect('equal')

        plt.tight_layout()
        plt.savefig(os.path.join(os.path.dirname(__file__), "output/uav_path_3d.png"),
                    dpi=150)
        print(f"\n路径图已保存到: output/uav_path_3d.png")

        plt.show()
    else:
        print("路径规划失败!")


if __name__ == "__main__":
    os.makedirs(os.path.join(os.path.dirname(__file__), "output"), exist_ok=True)
    main()
