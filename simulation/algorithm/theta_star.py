#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Theta* Path Planning Algorithm

Theta* 算法实现 - 真正的最短路径规划算法。
与 A* 的区别：支持"视距"扩展，获得真正最短路径。
"""

import heapq
import math
from typing import List, Tuple, Dict, Optional, Set
from dataclasses import dataclass, field
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from map_generator import Obstacle2D


@dataclass
class ThetaStarNode:
    """Theta* 路径节点"""
    x: int
    y: int
    g: float = field(default=0, repr=False)
    h: float = field(default=0, repr=False)
    f: float = field(default=0, repr=False)
    parent: Optional['ThetaStarNode'] = field(default=None, repr=False)

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __lt__(self, other):
        return self.f < other.f


class ThetaStar:
    """Theta* 路径规划器 - 真正的最短路径"""

    def __init__(self, obstacles: List[Obstacle2D],
                 grid_size: Tuple[int, int] = (50, 50),
                 resolution: float = 1.0,
                 heuristic_weight: float = 1.0):
        """
        初始化 Theta* 规划器

        Args:
            obstacles: 障碍物列表
            grid_size: 网格尺寸
            resolution: 分辨率
            heuristic_weight: 启发式权重
        """
        self.width = grid_size[0]
        self.height = grid_size[1]
        self.resolution = resolution
        self.obstacles = obstacles
        self.heuristic_weight = heuristic_weight

    def heuristic(self, node: Tuple[int, int],
                  goal: Tuple[int, int]) -> float:
        """
        欧几里得启发式（Theta* 使用欧几里得距离）
        """
        dx = abs(node[0] - goal[0])
        dy = abs(node[1] - goal[1])
        return math.sqrt(dx * dx + dy * dy)

    def is_collision(self, point: Tuple[float, float]) -> bool:
        """检查点是否与障碍物碰撞"""
        x, y = point[0], point[1]
        for obs in self.obstacles:
            if obs.contains((x, y)):
                return True
        return False

    def line_of_sight(self, p1: Tuple[float, float],
                      p2: Tuple[float, float],
                      check_interval: float = 0.25) -> bool:
        """
        视距检测 - 检查两点之间是否有障碍物

        使用 Bresenham 算法的改进版进行视距检测

        Args:
            p1: 起点坐标 (米)
            p2: 终点坐标 (米)
            check_interval: 检测间隔（米）

        Returns:
            True 表示两点可见（无障碍物阻挡）
        """
        x1, y1 = p1
        x2, y2 = p2

        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < 0.001:
            return True

        steps = max(int(distance / check_interval), 1)

        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * dx
            y = y1 + t * dy

            if self.is_collision((x, y)):
                return False

        return True

    def get_neighbors(self, node: ThetaStarNode) -> List[ThetaStarNode]:
        """获取邻居节点（8方向）"""
        neighbors = []
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]

        for dx, dy in directions:
            nx, ny = node.x + dx, node.y + dy

            if 0 <= nx < self.width and 0 <= ny < self.height:
                if not self.is_collision((nx, ny)):
                    neighbors.append(ThetaStarNode(x=nx, y=ny, g=float('inf')))

        return neighbors

    def calculate_cost(self, from_node: ThetaStarNode,
                       to_node: ThetaStarNode) -> float:
        """计算移动代价"""
        dx = abs(to_node.x - from_node.x)
        dy = abs(to_node.y - from_node.y)

        if dx == 1 and dy == 1:
            return math.sqrt(2)
        return 1.0

    def update_vertex(self, s: ThetaStarNode,
                      s_parent: Optional[ThetaStarNode],
                      goal_node: ThetaStarNode) -> None:
        """
        更新节点代价

        Theta* 的核心：检查 s 的父节点到 s 是否可见
        如果可见，直接使用父节点的 g 值
        """
        if s_parent is None:
            return

        s_parent_pos = (s_parent.x * self.resolution, s_parent.y * self.resolution)
        s_pos = (s.x * self.resolution, s.y * self.resolution)
        goal_pos = (goal_node.x * self.resolution, goal_node.y * self.resolution)

        if self.line_of_sight(s_parent_pos, s_pos):
            if s_parent.g + self._distance(s_parent_pos, s_pos) < s.g:
                s.parent = s_parent
                s.g = s_parent.g + self._distance(s_parent_pos, s_pos)
                s.h = self.heuristic((s.x, s.y), (goal_node.x, goal_node.y))
                s.f = s.g + s.h * self.heuristic_weight

    def _distance(self, p1: Tuple[float, float],
                  p2: Tuple[float, float]) -> float:
        """计算两点之间的欧几里得距离"""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.sqrt(dx * dx + dy * dy)

    def plan(self, start: Tuple[float, float],
             goal: Tuple[float, float]) -> Optional[Dict]:
        """
        执行 Theta* 路径规划

        Returns:
            路径数据字典，或 None
        """
        start_node = ThetaStarNode(
            x=int(start[0] / self.resolution),
            y=int(start[1] / self.resolution)
        )
        goal_node = ThetaStarNode(
            x=int(goal[0] / self.resolution),
            y=int(goal[1] / self.resolution)
        )

        if self.is_collision((start_node.x, start_node.y)):
            return None

        if self.is_collision((goal_node.x, goal_node.y)):
            return None

        open_set: List[ThetaStarNode] = []
        closed_set: Set[Tuple[int, int]] = set()

        start_node.g = 0
        start_node.h = self.heuristic(
            (start_node.x, start_node.y),
            (goal_node.x, goal_node.y)
        ) * self.heuristic_weight
        start_node.f = start_node.g + start_node.h
        heapq.heappush(open_set, start_node)

        iterations = 0
        max_iterations = 100000

        while open_set:
            current = heapq.heappop(open_set)

            if iterations > max_iterations:
                break

            iterations += 1

            if current.x == goal_node.x and current.y == goal_node.y:
                path = self._reconstruct_path(current)
                return {
                    "points": [{"x": p[0], "y": p[1]} for p in path],
                    "start": {"x": start[0], "y": start[1]},
                    "goal": {"x": goal[0], "y": goal[1]},
                    "algorithm": "Theta*",
                    "heuristic": "euclidean",
                    "iterations": iterations,
                    "path_length": self._calculate_path_length(path),
                    "num_waypoints": len(path)
                }

            if (current.x, current.y) in closed_set:
                continue

            closed_set.add((current.x, current.y))

            for neighbor in self.get_neighbors(current):
                if (neighbor.x, neighbor.y) in closed_set:
                    continue

                if neighbor.parent is None or neighbor.parent != current:
                    self.update_vertex(neighbor, current, goal_node)

                if current.g + self._calculate_cost(current, neighbor) < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = current.g + self._calculate_cost(current, neighbor)
                    neighbor.h = self.heuristic(
                        (neighbor.x, neighbor.y),
                        (goal_node.x, goal_node.y)
                    ) * self.heuristic_weight
                    neighbor.f = neighbor.g + neighbor.h
                    heapq.heappush(open_set, neighbor)

        return None

    def _calculate_cost(self, from_node: ThetaStarNode,
                        to_node: ThetaStarNode) -> float:
        """计算欧几里得移动代价"""
        dx = abs(to_node.x - from_node.x)
        dy = abs(to_node.y - from_node.y)
        return math.sqrt(dx * dx + dy * dy)

    def _reconstruct_path(self, end_node: ThetaStarNode) -> List[Tuple[float, float]]:
        """重建路径"""
        path = []
        current = end_node

        while current:
            path.append((current.x * self.resolution, current.y * self.resolution))
            current = current.parent

        return list(reversed(path))

    def _calculate_path_length(self, path: List[Tuple[float, float]]) -> float:
        """计算路径总长度"""
        total = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            total += math.sqrt(dx * dx + dy * dy)
        return total


def main():
    """测试 Theta* 规划器"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import json

    config_path = os.path.join(os.path.dirname(__file__), "../config/scenario_02.json")

    with open(config_path, 'r') as f:
        map_data = json.load(f)

    dog_map = map_data['dog_map']
    obstacles = [Obstacle2D.from_dict(obs) for obs in dog_map['obstacles']]

    start = (dog_map['start']['x'], dog_map['start']['y'])
    goal = (dog_map['goal']['x'], dog_map['goal']['y'])

    planner = ThetaStar(obstacles, (map_data['map_size']['x'], map_data['map_size']['y']))

    print("Testing Theta*...")
    result = planner.plan(start, goal)
    if result:
        print(f"  Path length: {result['path_length']:.2f}m")
        print(f"  Waypoints: {result['num_waypoints']}")
        print(f"  Iterations: {result['iterations']}")


if __name__ == "__main__":
    main()
