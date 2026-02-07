#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Optimized A* Path Planning Algorithm

优化的 A* 路径规划算法，支持多种启发式函数和参数配置。
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
class AStarNode:
    """A* 路径节点"""
    x: int
    y: int
    g: float = field(default=0, repr=False)
    h: float = field(default=0, repr=False)
    f: float = field(default=0, repr=False)
    parent: Optional['AStarNode'] = field(default=None, repr=False)

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __lt__(self, other):
        return self.f < other.f


class AStar:
    """优化的 A* 路径规划器"""

    def __init__(self, obstacles: List[Obstacle2D],
                 grid_size: Tuple[int, int] = (50, 50),
                 resolution: float = 1.0,
                 heuristic_weight: float = 1.0):
        """
        初始化 A* 规划器

        Args:
            obstacles: 障碍物列表
            grid_size: 网格尺寸 (width, height)
            resolution: 分辨率 (每个网格代表多少米)
            heuristic_weight: 启发式函数权重
        """
        self.width = grid_size[0]
        self.height = grid_size[1]
        self.resolution = resolution
        self.obstacles = obstacles
        self.heuristic_weight = heuristic_weight

    def heuristic(self, node: Tuple[int, int],
                  goal: Tuple[int, int],
                  method: str = "manhattan") -> float:
        """
        计算启发式代价

        Args:
            node: 当前节点
            goal: 目标节点
            method: 启发式方法 ('manhattan', 'euclidean', 'diagonal')

        Returns:
            启发式代价
        """
        dx = abs(node[0] - goal[0])
        dy = abs(node[1] - goal[1])

        if method == "manhattan":
            return dx + dy
        elif method == "euclidean":
            return math.sqrt(dx * dx + dy * dy)
        elif method == "diagonal":
            return max(dx, dy)
        elif method == "octile":
            return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
        return dx + dy

    def is_collision(self, point: Tuple[int, int]) -> bool:
        """检查点是否与障碍物碰撞"""
        x, y = point[0] * self.resolution, point[1] * self.resolution
        for obs in self.obstacles:
            if obs.contains((x, y)):
                return True
        return False

    def get_neighbors(self, node: AStarNode,
                       allow_diagonal: bool = True) -> List[AStarNode]:
        """
        获取邻居节点

        Args:
            node: 当前节点
            allow_diagonal: 是否允许对角线移动

        Returns:
            邻居节点列表
        """
        neighbors = []

        if allow_diagonal:
            directions = [
                (0, 1), (1, 0), (0, -1), (-1, 0),
                (1, 1), (1, -1), (-1, 1), (-1, -1)
            ]
        else:
            directions = [
                (0, 1), (1, 0), (0, -1), (-1, 0)
            ]

        for dx, dy in directions:
            nx, ny = node.x + dx, node.y + dy

            if 0 <= nx < self.width and 0 <= ny < self.height:
                if not self.is_collision((nx, ny)):
                    neighbors.append(AStarNode(x=nx, y=ny, g=float('inf')))

        return neighbors

    def calculate_cost(self, from_node: AStarNode,
                        to_node: AStarNode) -> float:
        """计算移动代价"""
        dx = abs(to_node.x - from_node.x)
        dy = abs(to_node.y - from_node.y)

        if dx == 1 and dy == 1:
            return math.sqrt(2)
        return 1.0

    def plan(self, start: Tuple[float, float],
             goal: Tuple[float, float],
             heuristic_method: str = "manhattan",
             allow_diagonal: bool = True,
             inflate_radius: int = 0) -> Optional[Dict]:
        """
        执行 A* 路径规划

        Args:
            start: 起点坐标 (x, y)
            goal: 终点坐标 (x, y)
            heuristic_method: 启发式方法
            allow_diagonal: 是否允许对角线移动
            inflate_radius: 障碍物膨胀半径

        Returns:
            路径数据字典，或 None
        """
        start_node = AStarNode(
            x=int(start[0] / self.resolution),
            y=int(start[1] / self.resolution)
        )
        goal_node = AStarNode(
            x=int(goal[0] / self.resolution),
            y=int(goal[1] / self.resolution)
        )

        if self.is_collision((start_node.x, start_node.y)):
            return None

        if self.is_collision((goal_node.x, goal_node.y)):
            return None

        open_set: List[AStarNode] = []
        closed_set: Set[Tuple[int, int]] = set()

        start_node.g = 0
        start_node.h = self.heuristic(
            (start_node.x, start_node.y),
            (goal_node.x, goal_node.y),
            heuristic_method
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
                    "algorithm": "A*",
                    "heuristic": heuristic_method,
                    "iterations": iterations,
                    "path_length": self._calculate_path_length(path),
                    "num_waypoints": len(path)
                }

            if (current.x, current.y) in closed_set:
                continue

            closed_set.add((current.x, current.y))

            for neighbor in self.get_neighbors(current, allow_diagonal):
                if (neighbor.x, neighbor.y) in closed_set:
                    continue

                tentative_g = current.g + self.calculate_cost(current, neighbor)

                if tentative_g < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = tentative_g
                    neighbor.h = self.heuristic(
                        (neighbor.x, neighbor.y),
                        (goal_node.x, goal_node.y),
                        heuristic_method
                    ) * self.heuristic_weight
                    neighbor.f = neighbor.g + neighbor.h
                    heapq.heappush(open_set, neighbor)

        return None

    def _reconstruct_path(self, end_node: AStarNode) -> List[Tuple[float, float]]:
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
    """测试 A* 规划器"""
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

    planner = AStar(obstacles, (map_data['map_size']['x'], map_data['map_size']['y']))

    print("Testing A* with different heuristics...")
    for method in ["manhattan", "euclidean", "diagonal"]:
        result = planner.plan(start, goal, heuristic_method=method)
        if result:
            print(f"  {method}: {result['path_length']:.2f}m, {result['num_waypoints']} points")


if __name__ == "__main__":
    main()
