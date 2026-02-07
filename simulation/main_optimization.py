#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 1 Optimization Main Program

A* 路径规划 + RDP简化 + B样条平滑
"""

import sys
import os
import json
import time
from typing import Dict, List
sys.path.insert(0, os.path.dirname(__file__))

from map_generator import Obstacle2D
from algorithm.a_star import AStar
from algorithm.path_optimizer import PathOptimizer, rdp_simplify, bspline_smooth
from evaluation.metrics import PathMetrics
from visualization.comparison import ComparisonVisualizer


def load_scenario(filepath: str) -> Dict:
    """加载场景配置"""
    with open(filepath, 'r') as f:
        return json.load(f)


def run_path_planning(config: Dict) -> Dict:
    """运行路径规划"""
    obstacles = [Obstacle2D.from_dict(obs) for obs in config['obstacles']]
    map_size = (config['map_size']['x'], config['map_size']['y'])
    start = (config['start']['x'], config['start']['y'])
    goal = (config['goal']['x'], config['goal']['y'])

    print(f"\n{'='*60}")
    print(f"场景: {config['name']}")
    print(f"{'='*60}")
    print(f"起点: {start}, 终点: {goal}")
    print(f"障碍物数量: {len(config['obstacles'])}")

    planner = AStar(obstacles, map_size)

    print("\n[运行 A* 规划...]")
    start_time = time.time()
    result = planner.plan(start, goal, heuristic_method="euclidean")
    planning_time = time.time() - start_time

    if result:
        metrics = PathMetrics.calculate_all(result, 0, planning_time)
        print(f"结果: {metrics['path_length']:.2f}m, {metrics['num_waypoints']} 点, 平滑度: {metrics['smoothness']:.3f}")
    else:
        print("规划失败!")
        metrics = None

    return {
        'config': config,
        'result': result,
        'metrics': metrics
    }


def run_path_optimization(raw_result: Dict) -> Dict:
    """运行路径优化"""
    if not raw_result.get('result'):
        return {}

    points = [(p['x'], p['y']) for p in raw_result['result']['points']]
    optimizer = PathOptimizer(rdp_epsilon=1.5, bspline_samples=50)
    optimized = optimizer.optimize(points)

    print(f"\n[路径优化结果]")
    print(f"  原始: {optimized['original']['num_points']} 点, {optimized['original']['length']:.2f}m, 平滑度: {optimized['original']['smoothness']:.3f}")
    print(f"  简化: {optimized['simplified']['num_points']} 点, {optimized['simplified']['length']:.2f}m, 平滑度: {optimized['simplified']['smoothness']:.3f}")
    print(f"  平滑: {optimized['smoothed']['num_points']} 点, {optimized['smoothed']['length']:.2f}m, 平滑度: {optimized['smoothed']['smoothness']:.3f}")

    return optimized


def generate_visualizations(raw_result: Dict, optimized: Dict):
    """生成可视化"""
    config = raw_result['config']
    scenario_name = config['name'].split(':')[0].split()[1].lower()[:6]

    visualizer = ComparisonVisualizer(config)

    print("\n[生成可视化...]")

    visualizer.plot_scenario_map(f"output/scenarios/{scenario_name}_map.png")

    if raw_result.get('result'):
        visualizer.plot_algorithm_comparison(
            raw_result['result'],
            None,
            f"output/algorithm_comparison/{scenario_name}_astar.png"
        )

    if optimized.get('original'):
        visualizer.plot_path_optimization(
            {'points': [{'x': p[0], 'y': p[1]} for p in optimized['original']['points']]},
            {'points': [{'x': p[0], 'y': p[1]} for p in optimized['simplified']['points']]},
            {'points': [{'x': p[0], 'y': p[1]} for p in optimized['smoothed']['points']]},
            f"output/path_optimization/{scenario_name}_optimization.png"
        )


def main():
    """主函数"""
    base_dir = os.path.dirname(__file__)

    scenarios = [
        os.path.join(base_dir, "config/scenarios/scenario_simple.json"),
        os.path.join(base_dir, "config/scenarios/scenario_maze.json"),
        os.path.join(base_dir, "config/scenarios/scenario_complex.json")
    ]

    print("=" * 70)
    print("Phase 1 优化: A* 路径规划 + RDP简化 + B样条平滑")
    print("=" * 70)

    all_results = []

    for scenario_path in scenarios:
        config = load_scenario(scenario_path)
        result = run_path_planning(config)
        optimized = run_path_optimization(result)
        generate_visualizations(result, optimized)
        all_results.append({
            'name': config['name'],
            'metrics': result['metrics'],
            'optimized': optimized
        })

    print("\n" + "=" * 70)
    print("测试结果汇总")
    print("=" * 70)

    for r in all_results:
        print(f"\n{r['name']}:")
        if r['metrics']:
            print(f"  A*: {r['metrics']['path_length']:.2f}m, {r['metrics']['num_waypoints']} 点, "
                  f"平滑度: {r['metrics']['smoothness']:.3f}, 时间: {r['metrics']['computation_time_ms']:.1f}ms")
        if r['optimized'].get('smoothed'):
            opt = r['optimized']['smoothed']
            print(f"  优化后: {opt['length']:.2f}m, {opt['num_points']} 点, 平滑度: {opt['smoothness']:.3f}")

    print("\n" + "=" * 70)
    print("所有可视化已保存到 output/")
    print("=" * 70)


if __name__ == "__main__":
    os.makedirs("output/scenarios", exist_ok=True)
    os.makedirs("output/algorithm_comparison", exist_ok=True)
    os.makedirs("output/path_optimization", exist_ok=True)
    os.makedirs("output/sensitivity_analysis", exist_ok=True)
    os.makedirs("output/performance", exist_ok=True)

    main()
