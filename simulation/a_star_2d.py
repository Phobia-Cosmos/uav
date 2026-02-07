#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2D A* Path Planning for Dog

机器狗 2D 路径规划算法实现。
"""

import heapq
import math
from typing import List, Tuple, Dict, Optional, Set
from dataclasses import dataclass, field
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from map_generator import Obstacle2D, load as load_map


@dataclass
class Node2D:
    """2D 路径节点"""
    x: int
    y: int
    g: float = field(default=0, repr=False)
    h: float = field(default=0, repr=False)
    f: float = field(default=0, repr=False)
    parent: Optional['Node2D'] = field(default=None, repr=False)

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __lt__(self, other):
        return self.f < other.f


class AStar2D:
    """2D A* 路径规划器"""

    def __init__(self, obstacles: List[Dict],
                 grid_size: Tuple[int, int] = (50, 50),
                 resolution: float = 1.0):
        """
        初始化 A* 规划器

        Args:
            obstacles: 障碍物列表 (字典格式)
            grid_size: 网格尺寸 (width, height)
            resolution: 分辨率 (每个网格代表多少米)
        """
        self.width = grid_size[0]
        self.height = grid_size[1]
        self.resolution = resolution
        self.obstacles = [Obstacle2D.from_dict(obs) for obs in obstacles]

    def heuristic(self, node: Tuple[int, int],
                  goal: Tuple[int, int]) -> float:
        """
        启发式函数 (曼哈顿距离)

        Args:
            node: 当前节点
            goal: 目标节点

        Returns:
            估计代价
        """
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    def is_collision(self, point: Tuple[int, int]) -> bool:
        """检查点是否与障碍物碰撞"""
        x, y = point[0] * self.resolution, point[1] * self.resolution

        for obs in self.obstacles:
            if obs.contains((x, y)):
                return True
        return False

    def get_neighbors(self, node: Node2D) -> List[Node2D]:
        """
        获取邻居节点 (8方向)

        Args:
            node: 当前节点

        Returns:
            邻居节点列表
        """
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # 上下左右
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # 对角线
        ]

        neighbors = []
        for dx, dy in directions:
            nx, ny = node.x + dx, node.y + dy

            # 检查边界
            if 0 <= nx < self.width and 0 <= ny < self.height:
                # 检查碰撞
                if not self.is_collision((nx, ny)):
                    neighbors.append(Node2D(x=nx, y=ny, g=float('inf')))

        return neighbors

    def calculate_cost(self, from_node: Node2D,
                      to_node: Node2D) -> float:
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

        if dx == 1 and dy == 1:
            # 对角线移动
            return math.sqrt(2)
        return 1.0

    def plan(self, start: Tuple[float, float],
             goal: Tuple[float, float],
             inflate_radius: int = 0) -> Optional[Dict]:
        """
        执行 A* 路径规划

        Args:
            start: 起点坐标 (x, y)
            goal: 终点坐标 (x, y)
            inflate_radius: 障碍物膨胀半径 (网格数)

        Returns:
            路径数据字典，或 None (如果无法规划)
        """
        # 转换为网格坐标
        start_node = Node2D(
            x=int(start[0] / self.resolution),
            y=int(start[1] / self.resolution)
        )
        goal_node = Node2D(
            x=int(goal[0] / self.resolution),
            y=int(goal[1] / self.resolution)
        )

        # 检查起点和终点是否有效
        if self.is_collision((start_node.x, start_node.y)):
            print(f"[A* 2D] 起点与障碍物碰撞!")
            return None

        if self.is_collision((goal_node.x, goal_node.y)):
            print(f"[A* 2D] 终点与障碍物碰撞!")
            return None

        # 膨胀障碍物
        inflated_obstacles = self._inflate_obstacles(inflate_radius)

        # A* 算法
        open_set = []
        closed_set: Set[Tuple[int, int]] = set()

        start_node.g = 0
        start_node.h = self.heuristic((start_node.x, start_node.y), (goal_node.x, goal_node.y))
        start_node.f = start_node.g + start_node.h
        heapq.heappush(open_set, start_node)

        while open_set:
            current = heapq.heappop(open_set)

            # 到达目标
            if current.x == goal_node.x and current.y == goal_node.y:
                path = self._reconstruct_path(current)
                return {
                    "points": [{"x": p[0], "y": p[1]} for p in path],
                    "start": {"x": start[0], "y": start[1]},
                    "goal": {"x": goal[0], "y": goal[1]},
                    "algorithm": "A* (Manhattan)",
                    "grid_size": self.width,
                    "inflate_radius": inflate_radius
                }

            # 跳过已访问节点
            if (current.x, current.y) in closed_set:
                continue

            closed_set.add((current.x, current.y))

            # 扩展邻居
            for neighbor in self.get_neighbors(current):
                if (neighbor.x, neighbor.y) in closed_set:
                    continue

                # 临时膨胀检查
                if inflated_obstacles and self._is_in_inflated(
                        neighbor.x, neighbor.y, inflated_obstacles):
                    continue

                tentative_g = current.g + self.calculate_cost(current, neighbor)

                if tentative_g < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = tentative_g
                    neighbor.h = self.heuristic(
                        (neighbor.x, neighbor.y),
                        (goal_node.x, goal_node.y)
                    )
                    neighbor.f = neighbor.g + neighbor.h
                    heapq.heappush(open_set, neighbor)

        print(f"[A* 2D] 未找到路径!")
        return None

    def _inflate_obstacles(self, radius: int) -> Set[Tuple[int, int]]:
        """膨胀障碍物"""
        if radius <= 0:
            return set()

        inflated = set()
        for obs in self.obstacles:
            if obs.type == "rectangle":
                width = int(obs.size["width"] / self.resolution) + 2 * radius
                height = int(obs.size["height"] / self.resolution) + 2 * radius
                ox = int(obs.position.x / self.resolution)
                oy = int(obs.position.y / self.resolution)

                for x in range(ox - width//2 - radius, ox + width//2 + radius + 1):
                    for y in range(oy - height//2 - radius, oy + height//2 + radius + 1):
                        if 0 <= x < self.width and 0 <= y < self.height:
                            inflated.add((x, y))

            elif obs.type == "circle":
                radius_grid = int(obs.size["radius"] / self.resolution) + radius
                ox = int(obs.position.x / self.resolution)
                oy = int(obs.position.y / self.resolution)

                for x in range(ox - radius_grid - radius, ox + radius_grid + radius + 1):
                    for y in range(oy - radius_grid - radius, oy + radius_grid + radius + 1):
                        if 0 <= x < self.width and 0 <= y < self.height:
                            if (x - ox)**2 + (y - oy)**2 <= (radius_grid + radius)**2:
                                inflated.add((x, y))

        return inflated

    def _is_in_inflated(self, x: int, y: int,
                        inflated: Set[Tuple[int, int]]) -> bool:
        """检查点是否在膨胀区域内"""
        return (x, y) in inflated

    def _reconstruct_path(self, end_node: Node2D) -> List[Tuple[float, float]]:
        """重建路径"""
        path = []
        current = end_node

        while current:
            path.append((current.x * self.resolution, current.y * self.resolution))
            current = current.parent

        return list(reversed(path))


def main():
    """测试 A* 2D 路径规划"""
    import matplotlib.pyplot as plt

    # 加载地图
    config_path = os.path.join(os.path.dirname(__file__), "config/scenario_01.json")
    map_data = load_map(config_path)

    print("=" * 50)
    print("    机器狗 2D 路径规划 (A*)")
    print("=" * 50)
    print(f"\n场景: {map_data['name']}")
    print(f"地图大小: {map_data['size']['x']}m x {map_data['size']['y']}m")
    print(f"障碍物数量: {len(map_data['dog_obstacles'])}")

    # 提取起点和终点
    start = (map_data['start']['x'], map_data['start']['y'])
    goal = (map_data['goal']['x'], map_data['goal']['y'])
    obstacles = map_data['dog_obstacles']

    print(f"\n起点: ({start[0]}, {start[1]})")
    print(f"终点: ({goal[0]}, {goal[1]})")

    # 创建规划器
    grid_size = (map_data['size']['x'], map_data['size']['y'])
    planner = AStar2D(obstacles, grid_size)

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
            total_length += math.sqrt(dx**2 + dy**2)

        print(f"路径总长度: {total_length:.2f}m")

        # 可视化
        fig, ax = plt.subplots(figsize=(10, 10))

        # 绘制障碍物
        for obs in obstacles:
            if obs['type'] == 'rectangle':
                x = obs['position']['x']
                y = obs['position']['y']
                w = obs['size']['width']
                h = obs['size']['height']
                rect = plt.Rectangle(
                    (x - w/2, y - h/2), w, h,
                    facecolor='lightgray', edgecolor='black'
                )
                ax.add_patch(rect)
            elif obs['type'] == 'circle':
                circle = plt.Circle(
                    (obs['position']['x'], obs['position']['y']),
                    obs['size']['radius'],
                    facecolor='lightgray', edgecolor='black'
                )
                ax.add_patch(circle)

        # 绘制起点和终点
        ax.scatter(start[0], start[1], c='green', s=200, marker='s',
                   label='Start', zorder=5)
        ax.scatter(goal[0], goal[1], c='red', s=200, marker='*',
                   label='Goal', zorder=5)

        # 绘制路径
        px = [p['x'] for p in path_result['points']]
        py = [p['y'] for p in path_result['points']]
        ax.plot(px, py, 'orange', linewidth=2, label='Dog Path')
        ax.scatter(px, py, c='orange', s=30, zorder=4)

        ax.set_title(f"Dog 2D Path Planning (A*)\nLength: {total_length:.2f}m", fontsize=14)
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        plt.tight_layout()
        plt.savefig(os.path.join(os.path.dirname(__file__), "output/dog_path_2d.png"),
                    dpi=150)
        print(f"\n路径图已保存到: output/dog_path_2d.png")

        plt.show()
    else:
        print("路径规划失败!")


if __name__ == "__main__":
    os.makedirs(os.path.join(os.path.dirname(__file__), "output"), exist_ok=True)
    main()
