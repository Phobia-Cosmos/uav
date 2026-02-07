#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ground-Air Cooperation Path Planning

地空协同路径规划主程序。
分别从机器狗视角和无人机视角进行2D路径规划与决策选优。
"""

import sys
import os
import math
import json
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Set, Dict
import heapq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(__file__))

from map_generator import load as load_map, Obstacle2D, Point2D


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

    def __init__(self, obstacles: list, grid_size: Tuple[int, int] = (50, 50),
                 resolution: float = 1.0):
        self.width = grid_size[0]
        self.height = grid_size[1]
        self.resolution = resolution
        self.obstacles = obstacles

    def heuristic(self, node: Tuple[int, int], goal: Tuple[int, int]) -> float:
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    def is_collision(self, point: Tuple[int, int]) -> bool:
        x, y = point[0] * self.resolution, point[1] * self.resolution
        for obs in self.obstacles:
            if obs.contains((x, y)):
                return True
        return False

    def get_neighbors(self, node: Node2D) -> List[Node2D]:
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]
        neighbors = []
        for dx, dy in directions:
            nx, ny = node.x + dx, node.y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if not self.is_collision((nx, ny)):
                    neighbors.append(Node2D(x=nx, y=ny, g=float('inf')))
        return neighbors

    def calculate_cost(self, from_node: Node2D, to_node: Node2D) -> float:
        dx = abs(to_node.x - from_node.x)
        dy = abs(to_node.y - from_node.y)
        if dx == 1 and dy == 1:
            return math.sqrt(2)
        return 1.0

    def plan(self, start: Tuple[float, float], goal: Tuple[float, float],
             inflate_radius: int = 0) -> Optional[Dict]:
        start_node = Node2D(
            x=int(start[0] / self.resolution),
            y=int(start[1] / self.resolution)
        )
        goal_node = Node2D(
            x=int(goal[0] / self.resolution),
            y=int(goal[1] / self.resolution)
        )

        if self.is_collision((start_node.x, start_node.y)):
            return None
        if self.is_collision((goal_node.x, goal_node.y)):
            return None

        open_set = []
        closed_set: Set[Tuple[int, int]] = set()

        start_node.g = 0
        start_node.h = self.heuristic((start_node.x, start_node.y), (goal_node.x, goal_node.y))
        start_node.f = start_node.g + start_node.h
        heapq.heappush(open_set, start_node)

        while open_set:
            current = heapq.heappop(open_set)

            if current.x == goal_node.x and current.y == goal_node.y:
                path = self._reconstruct_path(current)
                return {
                    "points": [{"x": p[0], "y": p[1]} for p in path],
                    "start": {"x": start[0], "y": start[1]},
                    "goal": {"x": goal[0], "y": goal[1]},
                    "algorithm": "A* (Manhattan)"
                }

            if (current.x, current.y) in closed_set:
                continue
            closed_set.add((current.x, current.y))

            for neighbor in self.get_neighbors(current):
                if (neighbor.x, neighbor.y) in closed_set:
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

        return None

    def _reconstruct_path(self, end_node: Node2D) -> List[Tuple[float, float]]:
        path = []
        current = end_node
        while current:
            path.append((current.x * self.resolution, current.y * self.resolution))
            current = current.parent
        return list(reversed(path))


def load_cooperation_map(filepath: str) -> Dict:
    """加载协同地图配置"""
    with open(filepath, 'r', encoding='utf-8') as f:
        return json.load(f)


def draw_obstacle_2d(ax, obs: Dict, color: str = '#FF6B6B', alpha: float = 0.8):
    """在2D图上绘制障碍物"""
    if obs['type'] == 'rectangle':
        x, y = obs['position']['x'], obs['position']['y']
        w, h = obs['size']['width'], obs['size']['height']
        rect = plt.Rectangle(
            (x - w/2, y - h/2), w, h,
            facecolor=color, edgecolor='black', linewidth=2, alpha=alpha
        )
        ax.add_patch(rect)
    elif obs['type'] == 'circle':
        x, y = obs['position']['x'], obs['position']['y']
        r = obs['size']['radius']
        circle = plt.Circle((x, y), r, facecolor=color, edgecolor='black', linewidth=2, alpha=alpha)
        ax.add_patch(circle)
    elif obs['type'] == 'wall':
        x1, y1 = obs['start_point']['x'], obs['start_point']['y']
        x2, y2 = obs['end_point']['x'], obs['end_point']['y']
        width = obs.get('width', 1)
        # 计算垂直于线段的方向
        dx, dy = x2 - x1, y2 - y1
        length = math.sqrt(dx*dx + dy*dy)
        if length > 0:
            nx, ny = -dy/length * width/2, dx/length * width/2
            polygon = plt.Polygon(
                [(x1+nx, y1+ny), (x2+nx, y2+ny), (x2-nx, y2-ny), (x1-nx, y1-ny)],
                facecolor='#8B4513', edgecolor='black', linewidth=2, alpha=0.9
            )
            ax.add_patch(polygon)


def calculate_path_length(points: List[Dict]) -> float:
    """计算路径总长度"""
    total = 0
    for i in range(1, len(points)):
        dx = points[i]['x'] - points[i-1]['x']
        dy = points[i]['y'] - points[i-1]['y']
        total += math.sqrt(dx*dx + dy*dy)
    return total


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    config_path = os.path.join(os.path.dirname(__file__), "config/scenario_02.json")

    print("=" * 70)
    print("    地空协同路径规划 (Ground-Air Cooperation Path Planning)")
    print("=" * 70)

    map_data = load_cooperation_map(config_path)
    map_size = (map_data['map_size']['x'], map_data['map_size']['y'])

    print(f"\n地图尺寸: {map_size[0]}m x {map_size[1]}m")
    print(f"场景: {map_data['name']}")
    print(f"描述: {map_data['description']}")

    dog_data = map_data['dog_map']
    uav_data = map_data['uav_map']

    dog_start = (dog_data['start']['x'], dog_data['start']['y'])
    dog_goal = (dog_data['goal']['x'], dog_data['goal']['y'])
    uav_start = (uav_data['start']['x'], uav_data['start']['y'])
    uav_goal = (uav_data['goal']['x'], uav_data['goal']['y'])

    print(f"\n[机器狗视角]")
    print(f"  起点: {dog_start}, 终点: {dog_goal}")
    print(f"  障碍物数量: {len(dog_data['obstacles'])}")

    print(f"\n[无人机视角]")
    print(f"  起点: {uav_start}, 终点: {uav_goal}")
    print(f"  障碍物数量: {len(uav_data['obstacles'])}")

    dog_obstacles = [Obstacle2D.from_dict(obs) for obs in dog_data['obstacles']]
    uav_obstacles = [Obstacle2D.from_dict(obs) for obs in uav_data['obstacles']]

    print("\n[运行路径规划...]")

    planner_dog = AStar2D(dog_obstacles, map_size)
    planner_uav = AStar2D(uav_obstacles, map_size)

    dog_path = planner_dog.plan(dog_start, dog_goal, inflate_radius=0)
    uav_path = planner_uav.plan(uav_start, uav_goal, inflate_radius=0)

    dog_length = calculate_path_length(dog_path['points']) if dog_path else float('inf')
    uav_length = calculate_path_length(uav_path['points']) if uav_path else float('inf')

    print(f"\n[路径规划结果]")
    print(f"  机器狗路径: {'找到' if dog_path else '未找到'} ({dog_length:.2f}m)")
    print(f"  无人机路径: {'找到' if uav_path else '未找到'} ({uav_length:.2f}m)")

    fig, axes = plt.subplots(1, 2, figsize=(20, 10))

    ax_dog = axes[0]
    ax_uav = axes[1]

    for obs in dog_data['obstacles']:
        draw_obstacle_2d(ax_dog, obs, color='#FF6B6B')

    ax_dog.scatter(dog_start[0], dog_start[1], c='green', s=300, marker='s',
                   zorder=5, label='Start', edgecolors='black', linewidths=2)
    ax_dog.scatter(dog_goal[0], dog_goal[1], c='red', s=300, marker='*',
                   zorder=5, label='Goal', edgecolors='black', linewidths=2)

    if dog_path:
        px = [p['x'] for p in dog_path['points']]
        py = [p['y'] for p in dog_path['points']]
        ax_dog.plot(px, py, 'b-', linewidth=3, label='Dog Path', alpha=0.8)
        ax_dog.scatter(px, py, c='blue', s=15, zorder=4)

    ax_dog.set_xlim(-2, 52)
    ax_dog.set_ylim(-2, 52)
    ax_dog.set_xlabel('X (meters)', fontsize=12)
    ax_dog.set_ylabel('Y (meters)', fontsize=12)
    ax_dog.set_title(f'Machine Dog View (Ground View)\\nObstacles: {len(dog_data["obstacles"])} | Path Length: {dog_length:.2f}m',
                     fontsize=14, fontweight='bold')
    ax_dog.legend(loc='upper left', fontsize=10)
    ax_dog.grid(True, alpha=0.3)
    ax_dog.set_aspect('equal')
    ax_dog.set_facecolor('#F5F5DC')

    for obs in uav_data['obstacles']:
        draw_obstacle_2d(ax_uav, obs, color='#4ECDC4')

    ax_uav.scatter(uav_start[0], uav_start[1], c='green', s=300, marker='s',
                   zorder=5, label='Start', edgecolors='black', linewidths=2)
    ax_uav.scatter(uav_goal[0], uav_goal[1], c='red', s=300, marker='*',
                   zorder=5, label='Goal', edgecolors='black', linewidths=2)

    if uav_path:
        px = [p['x'] for p in uav_path['points']]
        py = [p['y'] for p in uav_path['points']]
        ax_uav.plot(px, py, 'orange', linewidth=3, label='UAV Path', alpha=0.8)
        ax_uav.scatter(px, py, c='darkorange', s=15, zorder=4)

    ax_uav.set_xlim(-2, 52)
    ax_uav.set_ylim(-2, 52)
    ax_uav.set_xlabel('X (meters)', fontsize=12)
    ax_uav.set_ylabel('Y (meters)', fontsize=12)
    ax_uav.set_title(f'UAV View (Aerial View)\\nObstacles: {len(uav_data["obstacles"])} | Path Length: {uav_length:.2f}m',
                     fontsize=14, fontweight='bold')
    ax_uav.legend(loc='upper left', fontsize=10)
    ax_uav.grid(True, alpha=0.3)
    ax_uav.set_aspect('equal')
    ax_uav.set_facecolor('#E0F7FA')

    plt.suptitle(f'{map_data["name"]}\\nGround-Air Cooperation Path Planning Comparison',
                 fontsize=16, fontweight='bold', y=1.02)

    plt.tight_layout()
    output_path = os.path.join(os.path.dirname(__file__), "output/cooperation_paths.png")
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    print(f"\n[可视化已保存]")
    print(f"  {output_path}")

    print("\n" + "=" * 70)
    print("    决策选优 (Decision Optimization)")
    print("=" * 70)

    if dog_path and uav_path:
        print(f"\n路径比较:")
        print(f"  机器狗路径长度: {dog_length:.2f}m")
        print(f"  无人机路径长度: {uav_length:.2f}m")

        if uav_length < dog_length:
            print(f"\n推荐决策: 优先采用无人机路径 (节省 {(dog_length - uav_length):.2f}m)")
            decision = "UAV"
        else:
            print(f"\n推荐决策: 优先采用机器狗路径 (节省 {(uav_length - dog_length):.2f}m)")
            decision = "Dog"

        print(f"\n协同策略:")
        print(f"  1. 无人机可作为空中侦察，提前发现障碍物变化")
        print(f"  2. 机器狗可进行地面精确探测")
        print(f"  3. 双方路径可互相校验，提高安全性")


if __name__ == "__main__":
    os.makedirs(os.path.join(os.path.dirname(__file__), "output"), exist_ok=True)
    main()
