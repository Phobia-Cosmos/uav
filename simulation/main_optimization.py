#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 1 Optimization Main Program

A* Path Planning + RDP Simplification
"""

import sys
import os
import json
import time
from typing import Dict
sys.path.insert(0, os.path.dirname(__file__))

from map_generator import Obstacle2D
from algorithm.a_star import AStar
from algorithm.path_optimizer import rdp_simplify
from evaluation.metrics import PathMetrics
from visualization.comparison import PathVisualizer


def load_scenario(filepath: str) -> Dict:
    with open(filepath, 'r') as f:
        return json.load(f)


def run_path_planning(config: Dict) -> Dict:
    obstacles = [Obstacle2D.from_dict(obs) for obs in config['obstacles']]
    map_size = (config['map_size']['x'], config['map_size']['y'])
    start = (config['start']['x'], config['start']['y'])
    goal = (config['goal']['x'], config['goal']['y'])

    print(f"\n{'='*60}")
    print(f"Scenario: {config['name']}")
    print(f"{'='*60}")
    print(f"Start: {start}, Goal: {goal}")
    print(f"Obstacles: {len(config['obstacles'])}")

    planner = AStar(obstacles, map_size)

    print("\n[Running A*...]")
    start_time = time.time()
    result = planner.plan(start, goal, heuristic_method="euclidean")
    planning_time = time.time() - start_time

    if result:
        metrics = PathMetrics.calculate_all(result, 0, planning_time)
        print(f"Result: {metrics['path_length']:.2f}m, {metrics['num_waypoints']} pts, "
              f"smoothness: {metrics['smoothness']:.3f}, time: {metrics['computation_time_ms']:.1f}ms")
    else:
        print("Planning failed!")
        metrics = None

    return {
        'config': config,
        'result': result,
        'metrics': metrics
    }


def run_path_simplification(raw_result: Dict) -> Dict:
    if not raw_result.get('result'):
        return {}

    points = [(p['x'], p['y']) for p in raw_result['result']['points']]
    simplified = rdp_simplify(points, epsilon=1.5)

    original_length = sum(((points[i][0]-points[i-1][0])**2 + (points[i][1]-points[i-1][1])**2)**0.5 
                         for i in range(1, len(points)))
    simplified_length = sum(((simplified[i][0]-simplified[i-1][0])**2 + 
                             (simplified[i][1]-simplified[i-1][1])**2)**0.5 
                            for i in range(1, len(simplified)))

    print(f"\n[Path Simplification Results]")
    print(f"  Original: {len(points)} pts, {original_length:.2f}m")
    print(f"  Simplified: {len(simplified)} pts, {simplified_length:.2f}m")
    print(f"  Reduction: {len(points) - len(simplified)} pts ({100*(len(points)-len(simplified))/len(points):.1f}%)")

    return {
        'original': {'points': points, 'length': original_length},
        'simplified': {'points': simplified, 'length': simplified_length}
    }


def generate_visualizations(raw_result: Dict, simplified: Dict):
    config = raw_result['config']
    scenario_name = config['name'].split(':')[0].split()[1].lower()[:6]

    visualizer = PathVisualizer(config)

    print("\n[Generating visualizations...]")

    visualizer.plot_scenario_map(f"output/scenarios/{scenario_name}_map.png")

    if raw_result.get('result'):
        visualizer.plot_astar_path(
            raw_result['result'],
            f"output/paths/{scenario_name}_astar.png"
        )

    if simplified.get('original'):
        orig_pts = [{'x': p[0], 'y': p[1]} for p in simplified['original']['points']]
        simp_pts = [{'x': p[0], 'y': p[1]} for p in simplified['simplified']['points']]
        visualizer.plot_path_comparison(
            {'points': orig_pts},
            {'points': simp_pts},
            f"output/comparison/{scenario_name}_comparison.png"
        )


def generate_performance_chart(all_results: Dict, output_path: str):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    names = list(all_results.keys())

    lengths = [all_results[s]['metrics']['path_length'] if all_results[s]['metrics'] else 0 for s in names]
    points = [all_results[s]['metrics']['num_waypoints'] if all_results[s]['metrics'] else 0 for s in names]
    times = [all_results[s]['metrics']['computation_time_ms'] if all_results[s]['metrics'] else 0 for s in names]

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    axes[0].bar(names, lengths, color='#FF4500', alpha=0.8)
    axes[0].set_ylabel('Path Length (m)')
    axes[0].set_title('Path Length')

    axes[1].bar(names, points, color='#1E90FF', alpha=0.8)
    axes[1].set_ylabel('Number of Points')
    axes[1].set_title('Path Complexity')

    axes[2].bar(names, times, color='#32CD32', alpha=0.8)
    axes[2].set_ylabel('Time (ms)')
    axes[2].set_title('Computation Time')

    plt.suptitle('Path Planning Performance Comparison', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    print(f"Performance chart saved: {output_path}")


def main():
    base_dir = os.path.dirname(__file__)

    scenarios = [
        os.path.join(base_dir, "config/scenarios/scenario_simple.json"),
        os.path.join(base_dir, "config/scenarios/scenario_maze.json"),
        os.path.join(base_dir, "config/scenarios/scenario_complex.json")
    ]

    print("=" * 70)
    print("Phase 1: A* Path Planning + RDP Simplification")
    print("=" * 70)

    all_results = {}

    for scenario_path in scenarios:
        config = load_scenario(scenario_path)
        result = run_path_planning(config)
        simplified = run_path_simplification(result)
        generate_visualizations(result, simplified)

        all_results[config['name'].split(':')[0]] = {
            'metrics': result['metrics'],
            'simplified': simplified
        }

    generate_performance_chart(all_results, "output/performance.png")

    print("\n" + "=" * 70)
    print("Summary Results")
    print("=" * 70)

    for name, data in all_results.items():
        print(f"\n{name}:")
        if data['metrics']:
            m = data['metrics']
            print(f"  A*: {m['path_length']:.2f}m, {m['num_waypoints']} pts, "
                  f"smoothness: {m['smoothness']:.3f}, time: {m['computation_time_ms']:.1f}ms")
        if data['simplified']:
            s = data['simplified']['simplified']
            print(f"  Simplified: {len(s['points'])} pts")

    print("\n" + "=" * 70)
    print("Output Files:")
    print("  output/scenarios/    - Scenario maps")
    print("  output/paths/        - A* path visualizations")
    print("  output/comparison/   - Path comparison (original vs simplified)")
    print("  output/performance.png - Performance chart")
    print("=" * 70)


if __name__ == "__main__":
    os.makedirs("output/scenarios", exist_ok=True)
    os.makedirs("output/paths", exist_ok=True)
    os.makedirs("output/comparison", exist_ok=True)

    main()
