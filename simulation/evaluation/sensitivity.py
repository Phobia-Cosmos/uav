#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sensitivity Analysis Module

参数敏感性分析模块。
"""

import json
import time
import random
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from map_generator import Obstacle2D, load as load_map
from algorithm.a_star import AStar
from algorithm.theta_star import ThetaStar
from evaluation.metrics import PathMetrics


@dataclass
class SensitivityResult:
    """敏感性测试结果"""
    param_name: str
    param_values: List
    astar_results: List[Dict]
    theta_results: List[Dict]


class SensitivityAnalyzer:
    """参数敏感性分析器"""

    def __init__(self, scenario_config: Dict):
        """
        初始化分析器

        Args:
            scenario_config: 场景配置字典
        """
        self.config = scenario_config
        self.map_size = (scenario_config['map_size']['x'],
                         scenario_config['map_size']['y'])
        self.obstacles = [Obstacle2D.from_dict(obs)
                         for obs in scenario_config['obstacles']]
        self.start = (scenario_config['start']['x'],
                      scenario_config['start']['y'])
        self.goal = (scenario_config['goal']['x'],
                     scenario_config['goal']['y'])

    def test_grid_resolution(self,
                             resolutions: List[float] = [0.5, 1.0, 2.0, 5.0]) -> SensitivityResult:
        """
        测试网格分辨率敏感性

        Args:
            resolutions: 分辨率列表（米/格）

        Returns:
            测试结果
        """
        astar_results = []
        theta_results = []

        for res in resolutions:
            planner_a = AStar(self.obstacles, self.map_size, resolution=res)
            planner_t = ThetaStar(self.obstacles, self.map_size, resolution=res)

            start_time = time.time()
            astar_result = planner_a.plan(self.start, self.goal)
            astar_time = time.time() - start_time

            start_time = time.time()
            theta_result = planner_t.plan(self.start, self.goal)
            theta_time = time.time() - start_time

            astar_metrics = PathMetrics.calculate_all(
                astar_result if astar_result else {'points': []},
                0, astar_time
            ) if astar_result else {'path_length': float('inf'), 'num_waypoints': 0}

            theta_metrics = PathMetrics.calculate_all(
                theta_result if theta_result else {'points': []},
                0, theta_time
            ) if theta_result else {'path_length': float('inf'), 'num_waypoints': 0}

            astar_results.append(astar_metrics)
            theta_results.append(theta_metrics)

        return SensitivityResult(
            param_name='Grid Resolution (m)',
            param_values=resolutions,
            astar_results=astar_results,
            theta_results=theta_results
        )

    def test_heuristic_weight(self,
                              weights: List[float] = [0.5, 0.8, 1.0, 1.2, 1.5]) -> SensitivityResult:
        """
        测试启发式权重敏感性

        Args:
            weights: 权重列表

        Returns:
            测试结果
        """
        astar_results = []
        theta_results = []

        for weight in weights:
            planner_a = AStar(self.obstacles, self.map_size,
                             heuristic_weight=weight)
            planner_t = ThetaStar(self.obstacles, self.map_size,
                                 heuristic_weight=weight)

            start_time = time.time()
            astar_result = planner_a.plan(self.start, self.goal)
            astar_time = time.time() - start_time

            start_time = time.time()
            theta_result = planner_t.plan(self.start, self.goal)
            theta_time = time.time() - start_time

            astar_metrics = PathMetrics.calculate_all(
                astar_result if astar_result else {'points': []},
                0, astar_time
            ) if astar_result else {'path_length': float('inf'), 'num_waypoints': 0}

            theta_metrics = PathMetrics.calculate_all(
                theta_result if theta_result else {'points': []},
                0, theta_time
            ) if theta_result else {'path_length': float('inf'), 'num_waypoints': 0}

            astar_results.append(astar_metrics)
            theta_results.append(theta_metrics)

        return SensitivityResult(
            param_name='Heuristic Weight',
            param_values=weights,
            astar_results=astar_results,
            theta_results=theta_results
        )

    def test_obstacle_density(self,
                              densities: List[float] = [0.05, 0.10, 0.15, 0.20, 0.25],
                              num_tests: int = 5) -> Dict:
        """
        测试障碍物密度敏感性

        Args:
            densities: 密度列表（0-1）
            num_tests: 每个密度测试次数

        Returns:
            测试结果字典
        """
        results = {}

        for density in densities:
            test_results = []

            for _ in range(num_tests):
                test_obstacles = self._generate_random_obstacles(density)

                planner_a = AStar(test_obstacles, self.map_size)
                planner_t = ThetaStar(test_obstacles, self.map_size)

                start_time = time.time()
                astar_result = planner_a.plan(self.start, self.goal)
                astar_time = time.time() - start_time

                start_time = time.time()
                theta_result = planner_t.plan(self.start, self.goal)
                theta_time = time.time() - start_time

                astar_success = astar_result is not None
                theta_success = theta_result is not None

                test_results.append({
                    'astar_success': astar_success,
                    'theta_success': theta_success,
                    'astar_length': PathMetrics.path_length(astar_result['points']) if astar_result else float('inf'),
                    'theta_length': PathMetrics.path_length(theta_result['points']) if theta_result else float('inf'),
                    'astar_time': astar_time * 1000,
                    'theta_time': theta_time * 1000
                })

            results[density] = test_results

        return results

    def _generate_random_obstacles(self, density: float) -> List[Obstacle2D]:
        """生成随机障碍物"""
        obstacles = []
        base_area = self.map_size[0] * self.map_size[1]
        target_area = base_area * density

        current_area = 0
        max_obstacles = 50

        while current_area < target_area and len(obstacles) < max_obstacles:
            x = random.uniform(5, self.map_size[0] - 5)
            y = random.uniform(5, self.map_size[1] - 5)

            if random.random() < 0.5:
                width = random.uniform(2, 5)
                height = random.uniform(2, 5)
                obs = Obstacle2D(
                    id=f'obs_{len(obstacles)}',
                    type='rectangle',
                    position=None,
                    size={'width': width, 'height': height}
                )
                obs.position = type('Point', (), {'x': x, 'y': y})()
                current_area += width * height
            else:
                radius = random.uniform(1, 2.5)
                obs = Obstacle2D(
                    id=f'obs_{len(obstacles)}',
                    type='circle',
                    position=None,
                    size={'radius': radius}
                )
                obs.position = type('Point', (), {'x': x, 'y': y})()
                current_area += 3.14 * radius * radius

            obstacles.append(obs)

        return obstacles

    def run_full_analysis(self) -> Dict:
        """运行完整敏感性分析"""
        results = {
            'scenario': self.config['name'],
            'map_size': self.map_size,
            'start': self.start,
            'goal': self.goal,
            'grid_resolution': self.test_grid_resolution(),
            'heuristic_weight': self.test_heuristic_weight(),
            'obstacle_density': self.test_obstacle_density()
        }

        return results


def main():
    """测试敏感性分析"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    config_path = os.path.join(os.path.dirname(__file__),
                               '../config/scenario_02.json')

    with open(config_path, 'r') as f:
        config = json.load(f)

    print("Running sensitivity analysis...")
    analyzer = SensitivityAnalyzer(config['dog_map'])

    print("\n1. Testing grid resolution...")
    result = analyzer.test_grid_resolution()
    print(f"   Param: {result.param_values}")
    for i, res in enumerate(result.param_values):
        print(f"   {res}m: A*={result.astar_results[i]['path_length']:.2f}m, "
              f"Theta*={result.theta_results[i]['path_length']:.2f}m")

    print("\n2. Testing heuristic weight...")
    result = analyzer.test_heuristic_weight()
    print(f"   Param: {result.param_values}")
    for i, w in enumerate(result.param_values):
        print(f"   {w}: A*={result.astar_results[i]['path_length']:.2f}m, "
              f"Theta*={result.theta_results[i]['path_length']:.2f}m")


if __name__ == "__main__":
    main()
