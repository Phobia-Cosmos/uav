#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Report Charts Generator

生成报告所需的图表。
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from typing import Dict, List, Optional
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))


class ReportChartGenerator:
    """报告图表生成器"""

    def __init__(self):
        self.colors = {
            'astar': '#FF4500',
            'theta': '#1E90FF',
            'smoothed': '#32CD32'
        }

    def generate_summary_chart(self,
                             astar_metrics: Dict,
                             theta_metrics: Dict,
                             output_path: str):
        """生成综合摘要图表"""
        fig, ax = plt.subplots(figsize=(12, 6))

        metrics = ['Path Length\n(m)', 'Waypoints\n(count)', 'Turns\n(count)',
                  'Time\n(ms)', 'Smoothness\n(0-1)']

        astar_values = [
            astar_metrics.get('path_length', 0),
            astar_metrics.get('num_waypoints', 0),
            astar_metrics.get('num_turns', 0),
            astar_metrics.get('computation_time_ms', 0),
            astar_metrics.get('smoothness', 0)
        ]

        theta_values = [
            theta_metrics.get('path_length', 0),
            theta_metrics.get('num_waypoints', 0),
            theta_metrics.get('num_turns', 0),
            theta_metrics.get('computation_time_ms', 0),
            theta_metrics.get('smoothness', 0)
        ]

        x = np.arange(len(metrics))
        width = 0.35

        bars1 = ax.bar(x - width/2, astar_values, width, label='A*',
                       color=self.colors['astar'], alpha=0.8)
        bars2 = ax.bar(x + width/2, theta_values, width, label='Theta*',
                       color=self.colors['theta'], alpha=0.8)

        ax.set_ylabel('Value', fontsize=11)
        ax.set_title('Algorithm Performance Comparison', fontsize=14, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(metrics, fontsize=10)
        ax.legend(fontsize=11)
        ax.grid(True, alpha=0.3, axis='y')

        for bar in bars1:
            height = bar.get_height()
            ax.annotate(f'{height:.1f}',
                       xy=(bar.get_x() + bar.get_width() / 2, height),
                       xytext=(0, 3), textcoords="offset points",
                       ha='center', va='bottom', fontsize=8)

        for bar in bars2:
            height = bar.get_height()
            ax.annotate(f'{height:.1f}',
                       xy=(bar.get_x() + bar.get_width() / 2, height),
                       xytext=(0, 3), textcoords="offset points",
                       ha='center', va='bottom', fontsize=8)

        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()
        print(f"Summary chart saved: {output_path}")

    def generate_sensitivity_summary(self,
                                   results: Dict,
                                   output_path: str):
        """生成敏感性分析摘要图"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        if 'grid_resolution' in results:
            ax = axes[0, 0]
            res = results['grid_resolution']
            ax.plot(res['values'], [r['path_length'] for r in res['astar']],
                   'o-', color=self.colors['astar'], linewidth=2, label='A*')
            ax.plot(res['values'], [r['path_length'] for r in res['theta']],
                   's-', color=self.colors['theta'], linewidth=2, label='Theta*')
            ax.set_xlabel('Grid Resolution (m)')
            ax.set_ylabel('Path Length (m)')
            ax.set_title('Grid Resolution Impact', fontweight='bold')
            ax.legend()
            ax.grid(True, alpha=0.3)

        if 'heuristic_weight' in results:
            ax = axes[0, 1]
            hw = results['heuristic_weight']
            ax.plot(hw['values'], [r['path_length'] for r in hw['astar']],
                   'o-', color=self.colors['astar'], linewidth=2, label='A*')
            ax.plot(hw['values'], [r['path_length'] for r in hw['theta']],
                   's-', color=self.colors['theta'], linewidth=2, label='Theta*')
            ax.set_xlabel('Heuristic Weight')
            ax.set_ylabel('Path Length (m)')
            ax.set_title('Heuristic Weight Impact', fontweight='bold')
            ax.legend()
            ax.grid(True, alpha=0.3)

        if 'computation_time' in results:
            ax = axes[1, 0]
            ct = results['computation_time']
            ax.plot(ct['values'], [r['computation_time_ms'] for r in ct['astar']],
                   'o-', color=self.colors['astar'], linewidth=2, label='A*')
            ax.plot(ct['values'], [r['computation_time_ms'] for r in ct['theta']],
                   's-', color=self.colors['theta'], linewidth=2, label='Theta*')
            ax.set_xlabel('Map Size')
            ax.set_ylabel('Computation Time (ms)')
            ax.set_title('Computation Time vs Map Size', fontweight='bold')
            ax.legend()
            ax.grid(True, alpha=0.3)

        if 'smoothness' in results:
            ax = axes[1, 1]
            sm = results['smoothness']
            ax.bar(['A*', 'Theta*'], [sm['astar'], sm['theta']],
                  color=[self.colors['astar'], self.colors['theta']], alpha=0.8)
            ax.set_ylabel('Smoothness Score')
            ax.set_title('Path Smoothness Comparison', fontweight='bold')
            ax.grid(True, alpha=0.3, axis='y')
            ax.set_ylim(0, 1)

        plt.suptitle('Sensitivity Analysis Summary', fontsize=14, fontweight='bold', y=1.02)
        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()
        print(f"Sensitivity summary saved: {output_path}")


def main():
    """测试图表生成"""
    generator = ReportChartGenerator()

    astar = {
        'path_length': 72.5,
        'num_waypoints': 45,
        'num_turns': 12,
        'computation_time_ms': 25.3,
        'smoothness': 0.65
    }

    theta = {
        'path_length': 68.2,
        'num_waypoints': 38,
        'num_turns': 8,
        'computation_time_ms': 32.1,
        'smoothness': 0.78
    }

    generator.generate_summary_chart(astar, theta, "output/performance/summary_chart.png")


if __name__ == "__main__":
    main()
