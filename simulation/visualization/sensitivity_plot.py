#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sensitivity Analysis Visualization Module

参数敏感性分析图表生成模块。
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from typing import Dict, List, Optional
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))


class SensitivityPlotter:
    """参数敏感性分析图表生成器"""

    def __init__(self):
        self.colors = {
            'astar': '#FF4500',
            'theta': '#1E90FF'
        }

    def plot_resolution_sensitivity(self,
                                    resolutions: List[float],
                                    astar_lengths: List[float],
                                    theta_lengths: List[float],
                                    astar_times: List[float],
                                    theta_times: List[float],
                                    output_path: str):
        """绘制网格分辨率敏感性图"""
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))

        ax1, ax2 = axes

        ax1.plot(resolutions, astar_lengths, 'o-', color=self.colors['astar'],
                linewidth=2, markersize=8, label='A*')
        ax1.plot(resolutions, theta_lengths, 's-', color=self.colors['theta'],
                linewidth=2, markersize=8, label='Theta*')
        ax1.set_xlabel('Grid Resolution (m)', fontsize=11)
        ax1.set_ylabel('Path Length (m)', fontsize=11)
        ax1.set_title('Path Length vs Grid Resolution', fontsize=12, fontweight='bold')
        ax1.legend(fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.set_xlim(resolutions[0]-0.2, resolutions[-1]+0.2)

        ax2.plot(resolutions, astar_times, 'o-', color=self.colors['astar'],
                linewidth=2, markersize=8, label='A*')
        ax2.plot(resolutions, theta_times, 's-', color=self.colors['theta'],
                linewidth=2, markersize=8, label='Theta*')
        ax2.set_xlabel('Grid Resolution (m)', fontsize=11)
        ax2.set_ylabel('Computation Time (ms)', fontsize=11)
        ax2.set_title('Computation Time vs Grid Resolution', fontsize=12, fontweight='bold')
        ax2.legend(fontsize=10)
        ax2.grid(True, alpha=0.3)
        ax2.set_xlim(resolutions[0]-0.2, resolutions[-1]+0.2)

        plt.suptitle('Grid Resolution Sensitivity Analysis', fontsize=14, fontweight='bold', y=1.02)
        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()
        print(f"Resolution sensitivity saved: {output_path}")

    def plot_density_sensitivity(self,
                                 densities: List[float],
                                 astar_lengths: Dict[float, List],
                                 theta_lengths: Dict[float, List],
                                 output_path: str):
        """绘制障碍物密度敏感性图"""
        fig, ax = plt.subplots(figsize=(10, 6))

        astar_means = [np.mean(astar_lengths[d]) for d in densities]
        astar_stds = [np.std(astar_lengths[d]) for d in densities]
        theta_means = [np.mean(theta_lengths[d]) for d in densities]
        theta_stds = [np.std(theta_lengths[d]) for d in densities]

        x = np.arange(len(densities))
        width = 0.35

        bars1 = ax.bar(x - width/2, astar_means, width, yerr=astar_stds,
                      color=self.colors['astar'], label='A*', alpha=0.8, capsize=5)
        bars2 = ax.bar(x + width/2, theta_means, width, yerr=theta_stds,
                      color=self.colors['theta'], label='Theta*', alpha=0.8, capsize=5)

        ax.set_xlabel('Obstacle Density', fontsize=11)
        ax.set_ylabel('Path Length (m)', fontsize=11)
        ax.set_title('Path Length vs Obstacle Density (with std dev)', fontsize=12, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels([f'{d:.0%}' for d in densities])
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3, axis='y')

        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()
        print(f"Density sensitivity saved: {output_path}")

    def plot_weight_sensitivity(self,
                                 weights: List[float],
                                 astar_lengths: List[float],
                                 theta_lengths: List[float],
                                 output_path: str):
        """绘制启发式权重敏感性图"""
        fig, ax = plt.subplots(figsize=(10, 6))

        ax.plot(weights, astar_lengths, 'o-', color=self.colors['astar'],
               linewidth=2, markersize=8, label='A*')
        ax.plot(weights, theta_lengths, 's-', color=self.colors['theta'],
               linewidth=2, markersize=8, label='Theta*')

        ax.set_xlabel('Heuristic Weight', fontsize=11)
        ax.set_ylabel('Path Length (m)', fontsize=11)
        ax.set_title('Path Length vs Heuristic Weight', fontsize=12, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(weights[0]-0.1, weights[-1]+0.1)

        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()
        print(f"Weight sensitivity saved: {output_path}")

    def plot_performance_comparison(self,
                                     scenario_names: List[str],
                                     astar_metrics: List[Dict],
                                     theta_metrics: List[Dict],
                                     output_path: str):
        """绘制性能对比图"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        metrics_to_plot = [
            ('path_length', 'Path Length (m)', 'Path Length Comparison'),
            ('num_waypoints', 'Number of Waypoints', 'Path Complexity'),
            ('computation_time_ms', 'Computation Time (ms)', 'Computation Time'),
            ('smoothness', 'Smoothness Score (0-1)', 'Path Smoothness')
        ]

        x = np.arange(len(scenario_names))
        width = 0.35

        for idx, (metric_key, ylabel, title) in enumerate(metrics_to_plot):
            ax = axes[idx // 2, idx % 2]

            astar_values = [m.get(metric_key, 0) for m in astar_metrics]
            theta_values = [m.get(metric_key, 0) for m in theta_metrics]

            bars1 = ax.bar(x - width/2, astar_values, width,
                          color=self.colors['astar'], label='A*', alpha=0.8)
            bars2 = ax.bar(x + width/2, theta_values, width,
                          color=self.colors['theta'], label='Theta*', alpha=0.8)

            ax.set_xlabel('Scenario', fontsize=10)
            ax.set_ylabel(ylabel, fontsize=10)
            ax.set_title(title, fontsize=11, fontweight='bold')
            ax.set_xticks(x)
            ax.set_xticklabels([s.split(':')[0].split()[1] for s in scenario_names], fontsize=9)
            ax.legend(fontsize=9)
            ax.grid(True, alpha=0.3, axis='y')

        plt.suptitle('Algorithm Performance Comparison Across Scenarios',
                    fontsize=14, fontweight='bold', y=1.02)

        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()
        print(f"Performance comparison saved: {output_path}")


def main():
    """测试敏感性图表生成"""
    plotter = SensitivityPlotter()

    resolutions = [0.5, 1.0, 2.0, 5.0]
    astar_lengths = [68.5, 70.2, 73.8, 82.1]
    theta_lengths = [65.3, 66.8, 69.5, 78.2]
    astar_times = [45.2, 18.3, 6.1, 2.8]
    theta_times = [52.1, 21.5, 7.8, 3.2]

    plotter.plot_resolution_sensitivity(
        resolutions, astar_lengths, theta_lengths,
        astar_times, theta_times,
        "output/sensitivity_analysis/resolution_sensitivity.png"
    )


if __name__ == "__main__":
    main()
