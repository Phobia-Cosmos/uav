#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ground-Air Cooperation Path Planning

地空协同路径规划主程序。
分别从机器狗视角和无人机视角进行 2D 路径规划与决策选优。
"""

import json
import math
import os
import sys
from typing import Dict, List, Optional, Tuple

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(__file__))

from a_star_2d import AStar2D


def load_cooperation_map(filepath: str) -> Dict:
    with open(filepath, 'r', encoding='utf-8') as file:
        return json.load(file)


def draw_obstacle_2d(ax, obstacle: Dict, color: str = '#FF6B6B', alpha: float = 0.8):
    """在 2D 图上绘制障碍物。"""
    if obstacle['type'] == 'rectangle':
        x_pos, y_pos = obstacle['position']['x'], obstacle['position']['y']
        width, height = obstacle['size']['width'], obstacle['size']['height']
        rect = plt.Rectangle(
            (x_pos - width / 2, y_pos - height / 2),
            width,
            height,
            facecolor=color,
            edgecolor='black',
            linewidth=2,
            alpha=alpha,
        )
        ax.add_patch(rect)
    elif obstacle['type'] == 'circle':
        x_pos, y_pos = obstacle['position']['x'], obstacle['position']['y']
        radius = obstacle['size']['radius']
        circle = plt.Circle(
            (x_pos, y_pos),
            radius,
            facecolor=color,
            edgecolor='black',
            linewidth=2,
            alpha=alpha,
        )
        ax.add_patch(circle)
    elif obstacle['type'] == 'wall':
        x1, y1 = obstacle['start_point']['x'], obstacle['start_point']['y']
        x2, y2 = obstacle['end_point']['x'], obstacle['end_point']['y']
        width = obstacle.get('width', 1)
        dx, dy = x2 - x1, y2 - y1
        length = math.sqrt(dx * dx + dy * dy)
        if length > 0:
            nx, ny = -dy / length * width / 2, dx / length * width / 2
            polygon = plt.Polygon(
                [(x1 + nx, y1 + ny), (x2 + nx, y2 + ny), (x2 - nx, y2 - ny), (x1 - nx, y1 - ny)],
                facecolor='#8B4513',
                edgecolor='black',
                linewidth=2,
                alpha=0.9,
            )
            ax.add_patch(polygon)


def calculate_path_length(points: List[Dict]) -> float:
    total = 0.0
    for index in range(1, len(points)):
        delta_x = points[index]['x'] - points[index - 1]['x']
        delta_y = points[index]['y'] - points[index - 1]['y']
        total += math.sqrt(delta_x * delta_x + delta_y * delta_y)
    return total


def plot_agent_view(ax,
                    obstacles: List[Dict],
                    start: Tuple[float, float],
                    goal: Tuple[float, float],
                    path: Optional[Dict],
                    path_length: float,
                    map_size: Tuple[int, int],
                    obstacle_color: str,
                    path_color: str,
                    path_label: str,
                    title: str,
                    face_color: str):
    """绘制单个智能体视角。"""
    for obstacle in obstacles:
        draw_obstacle_2d(ax, obstacle, color=obstacle_color)

    ax.scatter(start[0], start[1], c='green', s=300, marker='s',
               zorder=5, label='Start', edgecolors='black', linewidths=2)
    ax.scatter(goal[0], goal[1], c='red', s=300, marker='*',
               zorder=5, label='Goal', edgecolors='black', linewidths=2)

    if path:
        path_x = [point['x'] for point in path['points']]
        path_y = [point['y'] for point in path['points']]
        ax.plot(path_x, path_y, color=path_color, linewidth=3, label=path_label, alpha=0.8)
        ax.scatter(path_x, path_y, c=path_color, s=15, zorder=4)

    ax.set_xlim(-2, map_size[0] + 2)
    ax.set_ylim(-2, map_size[1] + 2)
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.set_title(title.format(path_length=path_length, obstacle_count=len(obstacles)), fontsize=14, fontweight='bold')
    ax.legend(loc='upper left', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.set_facecolor(face_color)


def main():
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

    print("\n[机器狗视角]")
    print(f"  起点: {dog_start}, 终点: {dog_goal}")
    print(f"  障碍物数量: {len(dog_data['obstacles'])}")

    print("\n[无人机视角]")
    print(f"  起点: {uav_start}, 终点: {uav_goal}")
    print(f"  障碍物数量: {len(uav_data['obstacles'])}")

    print("\n[运行路径规划...]")
    planner_dog = AStar2D(dog_data['obstacles'], map_size)
    planner_uav = AStar2D(uav_data['obstacles'], map_size)

    dog_path = planner_dog.plan(dog_start, dog_goal, inflate_radius=0)
    uav_path = planner_uav.plan(uav_start, uav_goal, inflate_radius=0)

    dog_length = calculate_path_length(dog_path['points']) if dog_path else float('inf')
    uav_length = calculate_path_length(uav_path['points']) if uav_path else float('inf')

    print("\n[路径规划结果]")
    print(f"  机器狗路径: {'找到' if dog_path else '未找到'} ({dog_length:.2f}m)")
    print(f"  无人机路径: {'找到' if uav_path else '未找到'} ({uav_length:.2f}m)")

    fig, axes = plt.subplots(1, 2, figsize=(20, 10))
    plot_agent_view(
        axes[0],
        dog_data['obstacles'],
        dog_start,
        dog_goal,
        dog_path,
        dog_length,
        map_size,
        obstacle_color='#FF6B6B',
        path_color='blue',
        path_label='Dog Path',
        title='Machine Dog View (Ground View)\\nObstacles: {obstacle_count} | Path Length: {path_length:.2f}m',
        face_color='#F5F5DC',
    )
    plot_agent_view(
        axes[1],
        uav_data['obstacles'],
        uav_start,
        uav_goal,
        uav_path,
        uav_length,
        map_size,
        obstacle_color='#4ECDC4',
        path_color='darkorange',
        path_label='UAV Path',
        title='UAV View (Aerial View)\\nObstacles: {obstacle_count} | Path Length: {path_length:.2f}m',
        face_color='#E0F7FA',
    )

    plt.suptitle(
        f'{map_data["name"]}\\nGround-Air Cooperation Path Planning Comparison',
        fontsize=16,
        fontweight='bold',
        y=1.02,
    )

    plt.tight_layout()
    output_path = os.path.join(os.path.dirname(__file__), "output/cooperation_paths.png")
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    print("\n[可视化已保存]")
    print(f"  {output_path}")

    print("\n" + "=" * 70)
    print("    决策选优 (Decision Optimization)")
    print("=" * 70)

    if dog_path and uav_path:
        print("\n路径比较:")
        print(f"  机器狗路径长度: {dog_length:.2f}m")
        print(f"  无人机路径长度: {uav_length:.2f}m")

        if uav_length < dog_length:
            print(f"\n推荐决策: 优先采用无人机路径 (节省 {(dog_length - uav_length):.2f}m)")
        else:
            print(f"\n推荐决策: 优先采用机器狗路径 (节省 {(uav_length - dog_length):.2f}m)")

        print("\n协同策略:")
        print("  1. 无人机可作为空中侦察，提前发现障碍物变化")
        print("  2. 机器狗可进行地面精确探测")
        print("  3. 双方路径可互相校验，提高安全性")


if __name__ == "__main__":
    os.makedirs(os.path.join(os.path.dirname(__file__), "output"), exist_ok=True)
    main()
