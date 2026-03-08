#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Planning Visualization Module

A* 路径规划可视化。
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from typing import Dict, List
import argparse
import json
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from semantic_maps import get_physical_obstacles, obstacle_semantic_type


COLORS = {
    'wall': '#8B4513',
    'building': '#8D6E63',
    'debris': '#708090',
    'swamp_fill': '#4DB6AC',
    'swamp_edge': '#00695C',
    'crawl_fill': '#E0E0E0',
    'crawl_edge': '#616161',
    'start': '#00AA55',
    'goal': '#FF0000',
    'astar': '#FF4500',
    'background': '#F5F5DC'
}


def draw_obstacle(ax, obs: Dict):
    """绘制障碍物"""
    obs_type = obs.get('type', 'rectangle')
    semantic_type = obstacle_semantic_type(obs)

    if semantic_type in {'crawl_under_wall', 'low_wall', 'climbable_wall'}:
        style = {
            'facecolor': COLORS['crawl_fill'],
            'edgecolor': COLORS['crawl_edge'],
            'linewidth': 2,
            'alpha': 0.55,
            'linestyle': '--',
            'hatch': '///',
        }
    elif semantic_type in {'swamp', 'quicksand', 'water'}:
        style = {
            'facecolor': COLORS['swamp_fill'],
            'edgecolor': COLORS['swamp_edge'],
            'linewidth': 2,
            'alpha': 0.45,
            'linestyle': '-',
            'hatch': 'xx',
        }
    else:
        style = None

    if obs_type == 'rectangle':
        x, y = obs['position']['x'], obs['position']['y']
        w, h = obs['size']['width'], obs['size']['height']
        rect = plt.Rectangle(
            (x - w/2, y - h/2), w, h,
            facecolor=style['facecolor'] if style else COLORS.get('building'),
            edgecolor=style['edgecolor'] if style else 'black',
            linewidth=style['linewidth'] if style else 2,
            alpha=style['alpha'] if style else 0.8,
            linestyle=style['linestyle'] if style else '-',
            hatch=style['hatch'] if style else None,
        )
        ax.add_patch(rect)

    elif obs_type == 'circle':
        x, y = obs['position']['x'], obs['position']['y']
        r = obs['size']['radius']
        circle = plt.Circle(
            (x, y),
            r,
            facecolor=style['facecolor'] if style else COLORS.get('debris'),
            edgecolor=style['edgecolor'] if style else 'black',
            linewidth=style['linewidth'] if style else 2,
            alpha=style['alpha'] if style else 0.8,
            linestyle=style['linestyle'] if style else '-',
            hatch=style['hatch'] if style else None,
        )
        ax.add_patch(circle)

    elif obs_type == 'wall':
        x1, y1 = obs['start_point']['x'], obs['start_point']['y']
        x2, y2 = obs['end_point']['x'], obs['end_point']['y']
        width = obs.get('width', 1)

        dx, dy = x2 - x1, y2 - y1
        length = (dx**2 + dy**2)**0.5
        if length < 0.001:
            return

        nx, ny = -dy/length * width/2, dx/length * width/2
        polygon = plt.Polygon(
            [(x1+nx, y1+ny), (x2+nx, y2+ny), (x2-nx, y2-ny), (x1-nx, y1-ny)],
            facecolor=style['facecolor'] if style else COLORS.get('wall'),
            edgecolor=style['edgecolor'] if style else 'black',
            linewidth=style['linewidth'] if style else 2,
            alpha=style['alpha'] if style else 0.9,
            linestyle=style['linestyle'] if style else '-',
            hatch=style['hatch'] if style else None,
        )
        ax.add_patch(polygon)

    elif obs_type == 'polygon':
        polygon = plt.Polygon(
            [(vertex['x'], vertex['y']) for vertex in obs['vertices']],
            closed=True,
            facecolor=style['facecolor'] if style else COLORS.get('debris'),
            edgecolor=style['edgecolor'] if style else 'black',
            linewidth=style['linewidth'] if style else 2,
            alpha=style['alpha'] if style else 0.45,
            linestyle=style['linestyle'] if style else '-',
            hatch=style['hatch'] if style else None,
        )
        ax.add_patch(polygon)


class PathVisualizer:
    """路径可视化器"""

    def __init__(self, config: Dict):
        self.config = config
        self.map_size = (config['map_size']['x'], config['map_size']['y'])
        self.obstacles = get_physical_obstacles(config)
        if 'start' in config:
            self.start = (config['start']['x'], config['start']['y'])
            self.goal = (config['goal']['x'], config['goal']['y'])
        elif 'uav' in config:
            self.start = (config['uav']['start']['x'], config['uav']['start']['y'])
            self.goal = (config['uav']['goal']['x'], config['uav']['goal']['y'])
        else:
            self.start = (5, 5)
            self.goal = (45, 45)

    def plot_scenario_map(self, output_path: str):
        """绘制场景地图"""
        fig, ax = plt.subplots(figsize=(12, 10))

        for obs in self.obstacles:
            draw_obstacle(ax, obs)

        ax.scatter(self.start[0], self.start[1], c=COLORS['start'], s=300,
                  marker='s', zorder=5, label='Start', edgecolors='black', linewidths=2)
        ax.scatter(self.goal[0], self.goal[1], c=COLORS['goal'], s=300,
                  marker='*', zorder=5, label='Goal', edgecolors='black', linewidths=2)

        ax.set_xlim(-2, self.map_size[0] + 2)
        ax.set_ylim(-2, self.map_size[1] + 2)
        ax.set_xlabel('X (meters)', fontsize=12)
        ax.set_ylabel('Y (meters)', fontsize=12)
        ax.set_title(f"{self.config['name']}\nMap View", fontsize=14, fontweight='bold')
        ax.legend(loc='upper left', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        ax.set_facecolor(COLORS['background'])

        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()
        print(f"Scenario map saved: {output_path}")

    def plot_astar_path(self, astar_result: Dict, output_path: str):
        """绘制 A* 路径图"""
        fig, ax = plt.subplots(figsize=(12, 10))

        for obs in self.obstacles:
            draw_obstacle(ax, obs)

        ax.scatter(self.start[0], self.start[1], c=COLORS['start'], s=300,
                  marker='s', zorder=5, label='Start', edgecolors='black', linewidths=2)
        ax.scatter(self.goal[0], self.goal[1], c=COLORS['goal'], s=300,
                  marker='*', zorder=5, label='Goal', edgecolors='black', linewidths=2)

        if astar_result:
            apx = [p['x'] for p in astar_result['points']]
            apy = [p['y'] for p in astar_result['points']]
            ax.plot(apx, apy, '-', color=COLORS['astar'], linewidth=3,
                    label=f"A* Path ({astar_result.get('path_length', 0):.1f}m)")
            ax.scatter(apx, apy, c=COLORS['astar'], s=20, alpha=0.6)

        ax.set_xlim(-2, self.map_size[0] + 2)
        ax.set_ylim(-2, self.map_size[1] + 2)
        ax.set_xlabel('X (meters)', fontsize=12)
        ax.set_ylabel('Y (meters)', fontsize=12)
        ax.set_title(f"{self.config['name']}\nA* Path Planning", fontsize=14, fontweight='bold')
        ax.legend(loc='upper left', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        ax.set_facecolor(COLORS['background'])

        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()
        print(f"A* path saved: {output_path}")

    def plot_path_comparison(self, original: Dict, simplified: Dict, output_path: str):
        """绘制路径对比图（原始 vs 简化）"""
        fig, axes = plt.subplots(1, 2, figsize=(20, 8))

        COLORS_OPT = {
            'original': '#FF6B6B',
            'simplified': '#4ECDC4'
        }

        for ax in axes:
            for obs in self.obstacles:
                draw_obstacle(ax, obs)

            ax.scatter(self.start[0], self.start[1], c=COLORS['start'], s=200,
                      marker='s', zorder=5, edgecolors='black', linewidths=1)
            ax.scatter(self.goal[0], self.goal[1], c=COLORS['goal'], s=200,
                      marker='*', zorder=5, edgecolors='black', linewidths=1)

            ax.set_xlim(-2, self.map_size[0] + 2)
            ax.set_ylim(-2, self.map_size[1] + 2)
            ax.set_aspect('equal')
            ax.set_facecolor(COLORS['background'])
            ax.grid(True, alpha=0.3)

        ax1, ax2 = axes

        if original:
            px = [p['x'] for p in original['points']]
            py = [p['y'] for p in original['points']]
            length = sum(((px[i]-px[i-1])**2 + (py[i]-py[i-1])**2)**0.5 for i in range(1, len(px)))
            ax1.plot(px, py, '-', color=COLORS_OPT['original'], linewidth=3)
            ax1.scatter(px, py, c=COLORS_OPT['original'], s=25, alpha=0.8)
            ax1.set_title(f"Original Path\\n{len(original['points'])} pts, {length:.1f}m",
                         fontsize=12, fontweight='bold')

        if simplified:
            px = [p['x'] for p in simplified['points']]
            py = [p['y'] for p in simplified['points']]
            length = sum(((px[i]-px[i-1])**2 + (py[i]-py[i-1])**2)**0.5 for i in range(1, len(px)))
            ax2.plot(px, py, '-', color=COLORS_OPT['simplified'], linewidth=3)
            ax2.scatter(px, py, c=COLORS_OPT['simplified'], s=40, alpha=0.8)
            ax2.set_title(f"RDP Simplified Path\\n{len(simplified['points'])} pts, {length:.1f}m",
                         fontsize=12, fontweight='bold')

        plt.suptitle(f"{self.config['name']} - Path Optimization",
                    fontsize=14, fontweight='bold', y=1.02)

        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()
        print(f"Path comparison saved: {output_path}")


def parse_args() -> argparse.Namespace:
    base_dir = os.path.dirname(os.path.dirname(__file__))
    default_config = os.path.join(base_dir, "config", "scenarios", "scenario_a_simple.json")
    default_output = os.path.join(base_dir, "output", "scenarios", "simple_map.png")

    parser = argparse.ArgumentParser(description='路径规划可视化工具')
    parser.add_argument('--config', default=default_config, help='场景配置文件路径')
    parser.add_argument('--output', default=default_output, help='输出图片路径')
    return parser.parse_args()


def main():
    """测试可视化。"""
    args = parse_args()

    with open(args.config, 'r', encoding='utf-8') as file:
        config = json.load(file)

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    print("Testing visualization...")
    visualizer = PathVisualizer(config)
    visualizer.plot_scenario_map(args.output)


if __name__ == "__main__":
    main()
