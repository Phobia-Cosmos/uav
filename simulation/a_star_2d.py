#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2D A* Path Planning for Dog

机器狗 2D 路径规划算法实现。
"""

import heapq
import math
from typing import Dict, List, Optional, Set, Tuple
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from algorithm.a_star import AStar, AStarNode
from map_generator import Obstacle2D, load as load_map


class AStar2D(AStar):
    """2D A* 路径规划器。"""

    def __init__(self,
                 obstacles: List[Dict],
                 grid_size: Tuple[int, int] = (50, 50),
                 resolution: float = 1.0):
        obstacle_objects = [Obstacle2D.from_dict(obs) for obs in obstacles]
        super().__init__(obstacle_objects, grid_size=grid_size, resolution=resolution)

    def plan(self,
             start: Tuple[float, float],
             goal: Tuple[float, float],
             inflate_radius: int = 0) -> Optional[Dict]:
        """执行 2D A* 路径规划。"""
        start_node = AStarNode(
            x=int(start[0] / self.resolution),
            y=int(start[1] / self.resolution)
        )
        goal_node = AStarNode(
            x=int(goal[0] / self.resolution),
            y=int(goal[1] / self.resolution)
        )

        if self.is_collision((start_node.x, start_node.y)):
            print("[A* 2D] 起点与障碍物碰撞!")
            return None

        if self.is_collision((goal_node.x, goal_node.y)):
            print("[A* 2D] 终点与障碍物碰撞!")
            return None

        inflated_obstacles = self._inflate_obstacles(inflate_radius)

        open_set: List[AStarNode] = []
        closed_set: Set[Tuple[int, int]] = set()

        start_node.g = 0
        start_node.h = self.heuristic(
            (start_node.x, start_node.y),
            (goal_node.x, goal_node.y),
            "manhattan"
        )
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
                    "points": [{"x": point[0], "y": point[1]} for point in path],
                    "start": {"x": start[0], "y": start[1]},
                    "goal": {"x": goal[0], "y": goal[1]},
                    "algorithm": "A* (Manhattan)",
                    "heuristic": "manhattan",
                    "iterations": iterations,
                    "path_length": self._calculate_path_length(path),
                    "num_waypoints": len(path),
                    "grid_size": self.width,
                    "inflate_radius": inflate_radius,
                }

            if (current.x, current.y) in closed_set:
                continue

            closed_set.add((current.x, current.y))

            for neighbor in self.get_neighbors(current, allow_diagonal=True):
                if (neighbor.x, neighbor.y) in closed_set:
                    continue

                if inflated_obstacles and self._is_in_inflated(
                    neighbor.x, neighbor.y, inflated_obstacles
                ):
                    continue

                tentative_g = current.g + self.calculate_cost(current, neighbor)
                if tentative_g < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = tentative_g
                    neighbor.h = self.heuristic(
                        (neighbor.x, neighbor.y),
                        (goal_node.x, goal_node.y),
                        "manhattan"
                    )
                    neighbor.f = neighbor.g + neighbor.h
                    heapq.heappush(open_set, neighbor)

        print("[A* 2D] 未找到路径!")
        return None

    def _inflate_obstacles(self, radius: int) -> Set[Tuple[int, int]]:
        """膨胀障碍物。"""
        if radius <= 0:
            return set()

        inflated: Set[Tuple[int, int]] = set()
        for obstacle in self.obstacles:
            if obstacle.type == "rectangle":
                width = int(obstacle.size["width"] / self.resolution) + 2 * radius
                height = int(obstacle.size["height"] / self.resolution) + 2 * radius
                obstacle_x = int(obstacle.position.x / self.resolution)
                obstacle_y = int(obstacle.position.y / self.resolution)

                for x in range(obstacle_x - width // 2 - radius, obstacle_x + width // 2 + radius + 1):
                    for y in range(obstacle_y - height // 2 - radius, obstacle_y + height // 2 + radius + 1):
                        if 0 <= x < self.width and 0 <= y < self.height:
                            inflated.add((x, y))

            elif obstacle.type == "circle":
                radius_grid = int(obstacle.size["radius"] / self.resolution) + radius
                obstacle_x = int(obstacle.position.x / self.resolution)
                obstacle_y = int(obstacle.position.y / self.resolution)

                for x in range(obstacle_x - radius_grid - radius, obstacle_x + radius_grid + radius + 1):
                    for y in range(obstacle_y - radius_grid - radius, obstacle_y + radius_grid + radius + 1):
                        if 0 <= x < self.width and 0 <= y < self.height:
                            if (x - obstacle_x) ** 2 + (y - obstacle_y) ** 2 <= (radius_grid + radius) ** 2:
                                inflated.add((x, y))

            elif obstacle.type == "wall":
                if obstacle.end_point is None:
                    continue

                min_x = int(min(obstacle.position.x, obstacle.end_point.x) / self.resolution) - radius - 1
                max_x = int(max(obstacle.position.x, obstacle.end_point.x) / self.resolution) + radius + 1
                min_y = int(min(obstacle.position.y, obstacle.end_point.y) / self.resolution) - radius - 1
                max_y = int(max(obstacle.position.y, obstacle.end_point.y) / self.resolution) + radius + 1

                wall_margin = obstacle.size.get("width", 1) / 2 + radius * self.resolution
                for x in range(max(0, min_x), min(self.width, max_x + 1)):
                    for y in range(max(0, min_y), min(self.height, max_y + 1)):
                        world_x = x * self.resolution
                        world_y = y * self.resolution
                        if obstacle._point_to_segment_distance(world_x, world_y) <= wall_margin:
                            inflated.add((x, y))

        return inflated

    def _is_in_inflated(self, x: int, y: int, inflated: Set[Tuple[int, int]]) -> bool:
        return (x, y) in inflated


def main():
    """测试 A* 2D 路径规划。"""
    import matplotlib.pyplot as plt

    config_path = os.path.join(os.path.dirname(__file__), "config/scenario_01.json")
    map_data = load_map(config_path)

    print("=" * 50)
    print("    机器狗 2D 路径规划 (A*)")
    print("=" * 50)
    print(f"\n场景: {map_data['name']}")
    print(f"地图大小: {map_data['size']['x']}m x {map_data['size']['y']}m")
    print(f"障碍物数量: {len(map_data['dog_obstacles'])}")

    start = (map_data['start']['x'], map_data['start']['y'])
    goal = (map_data['goal']['x'], map_data['goal']['y'])
    obstacles = map_data['dog_obstacles']

    print(f"\n起点: ({start[0]}, {start[1]})")
    print(f"终点: ({goal[0]}, {goal[1]})")

    grid_size = (map_data['size']['x'], map_data['size']['y'])
    planner = AStar2D(obstacles, grid_size)

    print("\n[运行 A* 算法...]")
    path_result = planner.plan(start, goal, inflate_radius=1)

    if not path_result:
        print("路径规划失败!")
        return

    print(f"路径找到! 点数: {len(path_result['points'])}")
    total_length = path_result.get('path_length', 0.0)
    print(f"路径总长度: {total_length:.2f}m")

    fig, ax = plt.subplots(figsize=(10, 10))

    for obstacle in obstacles:
        if obstacle['type'] == 'rectangle':
            x = obstacle['position']['x']
            y = obstacle['position']['y']
            width = obstacle['size']['width']
            height = obstacle['size']['height']
            rect = plt.Rectangle(
                (x - width / 2, y - height / 2),
                width,
                height,
                facecolor='lightgray',
                edgecolor='black'
            )
            ax.add_patch(rect)
        elif obstacle['type'] == 'circle':
            circle = plt.Circle(
                (obstacle['position']['x'], obstacle['position']['y']),
                obstacle['size']['radius'],
                facecolor='lightgray',
                edgecolor='black'
            )
            ax.add_patch(circle)

    ax.scatter(start[0], start[1], c='green', s=200, marker='s', label='Start', zorder=5)
    ax.scatter(goal[0], goal[1], c='red', s=200, marker='*', label='Goal', zorder=5)

    path_x = [point['x'] for point in path_result['points']]
    path_y = [point['y'] for point in path_result['points']]
    ax.plot(path_x, path_y, 'orange', linewidth=2, label='Dog Path')
    ax.scatter(path_x, path_y, c='orange', s=30, zorder=4)

    ax.set_title(f"Dog 2D Path Planning (A*)\nLength: {total_length:.2f}m", fontsize=14)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    plt.tight_layout()
    output_path = os.path.join(os.path.dirname(__file__), "output/dog_path_2d.png")
    plt.savefig(output_path, dpi=150)
    print(f"\n路径图已保存到: {output_path}")
    plt.show()


if __name__ == "__main__":
    os.makedirs(os.path.join(os.path.dirname(__file__), "output"), exist_ok=True)
    main()
