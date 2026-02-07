#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Comparison Visualization Module

A* vs Theta* 路径对比可视化。
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from typing import Dict, List, Tuple, Optional
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from map_generator import Obstacle2D


COLORS = {
    'wall': '#8B4513',
    'building': '#CD853F',
    'debris': '#708090',
    'tree': '#228B22',
    'start': '#00FF00',
    'goal': '#FF0000',
    'astar': '#FF4500',
    'theta': '#1E90FF',
    'background': '#F5F5DC'
}


def draw_obstacle(ax, obs: Dict, color: str = None):
    """绘制障碍物"""
    obs_type = obs.get('type', 'rectangle')

    if obs_type == 'rectangle':
        x, y = obs['position']['x'], obs['position']['y']
        w, h = obs['size']['width'], obs['size']['height']
        rect = plt.Rectangle(
            (x - w/2, y - h/2), w, h,
            facecolor=COLORS.get('building', color),
            edgecolor='black', linewidth=2, alpha=0.8
        )
        ax.add_patch(rect)

    elif obs_type == 'circle':
        x, y = obs['position']['x'], obs['position']['y']
        r = obs['size']['radius']
        circle = plt.Circle((x, y), r, facecolor=COLORS.get('debris', color),
                           edgecolor='black', linewidth=2, alpha=0.8)
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
            facecolor=COLORS.get('wall', color), edgecolor='black',
            linewidth=2, alpha=0.9
        )
        ax.add_patch(polygon)


class ComparisonVisualizer:
    """A* vs Theta* 对比可视化器"""

    def __init__(self, config: Dict):
        self.config = config
        self.map_size = (config['map_size']['x'], config['map_size']['y'])
        self.obstacles = config['obstacles']
        self.start = (config['start']['x'], config['start']['y'])
        self.goal = (config['goal']['x'], config['goal']['y'])

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

    def plot_algorithm_comparison(self,
                                  astar_result: Dict,
                                  theta_result: Dict,
                                  output_path: str):
        """绘制算法对比图"""
        fig, axes = plt.subplots(1, 3, figsize=(20, 7))

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

        ax1, ax2, ax3 = axes

        if astar_result:
            apx = [p['x'] for p in astar_result['points']]
            apy = [p['y'] for p in astar_result['points']]
            ax1.plot(apx, apy, '-', color=COLORS['astar'], linewidth=3,
                    label=f"A* Path ({astar_result.get('path_length', 0):.1f}m)")
            ax1.scatter(apx, apy, c=COLORS['astar'], s=15, alpha=0.6)
            ax1.set_title('A* Algorithm', fontsize=12, fontweight='bold')
            ax1.legend(loc='upper left', fontsize=9)

        if theta_result:
            tpx = [p['x'] for p in theta_result['points']]
            tpy = [p['y'] for p in theta_result['points']]
            ax2.plot(tpx, tpy, '-', color=COLORS['theta'], linewidth=3,
                    label=f"Theta* Path ({theta_result.get('path_length', 0):.1f}m)")
            ax2.scatter(tpx, tpy, c=COLORS['theta'], s=15, alpha=0.6)
            ax2.set_title('Theta* Algorithm', fontsize=12, fontweight='bold')
            ax2.legend(loc='upper left', fontsize=9)

        if astar_result and theta_result:
            ax3.plot(apx, apy, '-', color=COLORS['astar'], linewidth=2,
                    label=f"A* ({astar_result.get('path_length', 0):.1f}m)", alpha=0.7)
            ax3.plot(tpx, tpy, '-', color=COLORS['theta'], linewidth=2,
                    label=f"Theta* ({theta_result.get('path_length', 0):.1f}m)", alpha=0.7)
            ax3.set_title('Overlay Comparison', fontsize=12, fontweight='bold')
            ax3.legend(loc='upper left', fontsize=9)

        plt.suptitle(f"{self.config['name']} - Algorithm Comparison",
                    fontsize=14, fontweight='bold', y=1.02)

        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()
        print(f"Algorithm comparison saved: {output_path}")

    def plot_path_optimization(self,
                               original: Dict,
                               simplified: Dict,
                               smoothed: Dict,
                               output_path: str):
        """绘制路径优化对比图"""
        fig, axes = plt.subplots(1, 3, figsize=(20, 7))

        COLORS_OPT = {
            'original': '#FF6B6B',
            'simplified': '#4ECDC4',
            'smoothed': '#45B7D1'
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

        ax1, ax2, ax3 = axes

        if original:
            px = [p['x'] for p in original['points']]
            py = [p['y'] for p in original['points']]
            length = sum(((px[i]-px[i-1])**2 + (py[i]-py[i-1])**2)**0.5 for i in range(1, len(px)))
            ax1.plot(px, py, '-', color=COLORS_OPT['original'], linewidth=3)
            ax1.scatter(px, py, c=COLORS_OPT['original'], s=20, alpha=0.8)
            ax1.set_title(f"Original\\n{len(original['points'])} pts, {length:.1f}m",
                         fontsize=11, fontweight='bold')

        if simplified:
            px = [p['x'] for p in simplified['points']]
            py = [p['y'] for p in simplified['points']]
            length = sum(((px[i]-px[i-1])**2 + (py[i]-py[i-1])**2)**0.5 for i in range(1, len(px)))
            ax2.plot(px, py, '-', color=COLORS_OPT['simplified'], linewidth=3)
            ax2.scatter(px, py, c=COLORS_OPT['simplified'], s=30, alpha=0.8)
            ax2.set_title(f"RDP Simplified\\n{len(simplified['points'])} pts, {length:.1f}m",
                         fontsize=11, fontweight='bold')

        if smoothed:
            px = [p['x'] for p in smoothed['points']]
            py = [p['y'] for p in smoothed['points']]
            length = sum(((px[i]-px[i-1])**2 + (py[i]-py[i-1])**2)**0.5 for i in range(1, len(px)))
            ax3.plot(px, py, '-', color=COLORS_OPT['smoothed'], linewidth=3)
            ax3.scatter(px, py, c=COLORS_OPT['smoothed'], s=30, alpha=0.8)
            ax3.set_title(f"B-Spline Smoothed\\n{len(smoothed['points'])} pts, {length:.1f}m",
                         fontsize=11, fontweight='bold')

        plt.suptitle(f"{self.config['name']} - Path Optimization",
                    fontsize=14, fontweight='bold', y=1.02)

        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()
        print(f"Path optimization saved: {output_path}")


def main():
    """测试可视化"""
    import json

    config_path = "../config/scenarios/scenario_maze.json"
    with open(config_path, 'r') as f:
        config = json.load(f)

    print("Testing comparison visualization...")
    visualizer = ComparisonVisualizer(config)
    visualizer.plot_scenario_map("output/scenarios/maze_map.png")


if __name__ == "__main__":
    main()
