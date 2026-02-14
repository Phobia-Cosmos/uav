#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UAV vs Dog Path Comparison

Dog and UAV have different obstacle maps based on their perspectives.
Dog cannot see hidden passages that UAV can fly over.
"""

import sys
import os
import json
import time
from typing import Dict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(__file__))

from map_generator import Obstacle2D
from algorithm.a_star import AStar
from algorithm.path_optimizer import rdp_simplify
from evaluation.metrics import PathMetrics


COLORS = {
    'wall': '#8B4513',
    'building': '#CD853F',
    'debris': '#708090',
    'tree': '#228B22',
    'start': '#00FF00',
    'goal': '#FF0000',
    'uav': '#0066CC',
    'dog': '#FF6600',
    'background': '#F5F5DC',
    'hidden_passage': '#AAAAAA'
}


def draw_obstacle(ax, obs: Dict):
    obs_type = obs.get('type', 'rectangle')

    if obs_type == 'rectangle':
        x, y = obs['position']['x'], obs['position']['y']
        w, h = obs['size']['width'], obs['size']['height']
        rect = plt.Rectangle(
            (x - w/2, y - h/2), w, h,
            facecolor=COLORS.get('building'),
            edgecolor='black', linewidth=2, alpha=0.8
        )
        ax.add_patch(rect)

    elif obs_type == 'circle':
        x, y = obs['position']['x'], obs['position']['y']
        r = obs['size']['radius']
        circle = plt.Circle((x, y), r, facecolor=COLORS.get('debris'),
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
            facecolor=COLORS.get('wall'), edgecolor='black',
            linewidth=2, alpha=0.9
        )
        ax.add_patch(polygon)


def load_scenario(filepath: str) -> Dict:
    with open(filepath, 'r') as f:
        return json.load(f)


def run_path_planning(obstacles: list, map_size: tuple, start: tuple, goal: tuple, agent_type: str) -> Dict:
    print(f"\n  [{agent_type}] Start: {start}, Goal: {goal}")
    print(f"  [{agent_type}] Obstacles: {len(obstacles)}")

    obs_objects = [Obstacle2D.from_dict(obs) for obs in obstacles]
    planner = AStar(obs_objects, map_size)

    start_time = time.time()
    result = planner.plan(start, goal, heuristic_method="euclidean")
    planning_time = time.time() - start_time

    if result:
        metrics = PathMetrics.calculate_all(result, 0, planning_time)
        print(f"  [{agent_type}] Path: {metrics['path_length']:.2f}m, {metrics['num_waypoints']} pts, "
              f"smoothness: {metrics['smoothness']:.3f}, time: {metrics['computation_time_ms']:.1f}ms")
    else:
        print(f"  [{agent_type}] Planning failed!")
        metrics = None

    return {
        'result': result,
        'metrics': metrics,
        'start': start,
        'goal': goal
    }


def run_simplification(raw_result: Dict, agent_type: str) -> Dict:
    if not raw_result.get('result'):
        return {}

    points = [(p['x'], p['y']) for p in raw_result['result']['points']]
    simplified = rdp_simplify(points, epsilon=1.5)

    original_length = sum(((points[i][0]-points[i-1][0])**2 + (points[i][1]-points[i-1][1])**2)**0.5
                         for i in range(1, len(points)))
    simplified_length = sum(((simplified[i][0]-simplified[i-1][0])**2 +
                             (simplified[i][1]-simplified[i-1][1])**2)**0.5
                            for i in range(1, len(simplified)))

    print(f"  [{agent_type}] Simplified: {len(points)} -> {len(simplified)} pts "
          f"({100*(len(points)-len(simplified))/len(points):.1f}% reduction)")

    return {
        'original': {'points': points, 'length': original_length},
        'simplified': {'points': simplified, 'length': simplified_length}
    }


def generate_map_image(config: Dict, obstacles: list, output_path: str, agent_type: str):
    """为单个场景生成地图图片"""
    map_size = (config['map_size']['x'], config['map_size']['y'])

    fig, ax = plt.subplots(figsize=(10, 9))

    for obs in obstacles:
        draw_obstacle(ax, obs)

    if agent_type == 'uav':
        start = config['uav']['start']
        goal = config['uav']['goal']
        title = f"UAV View: {config['name']}\n({len(obstacles)} obstacles visible to UAV)"
    else:
        start = config['dog']['start']
        goal = config['dog']['goal']
        title = f"Dog View: {config['name']}\n({len(obstacles)} obstacles visible to Dog)"

    ax.scatter(start['x'], start['y'], c=COLORS['start'], s=300, marker='s', zorder=5,
               label='Start', edgecolors='black', linewidths=2)
    ax.scatter(goal['x'], goal['y'], c=COLORS['goal'], s=300, marker='*', zorder=5,
               label='Goal', edgecolors='black', linewidths=2)

    ax.set_xlim(-2, map_size[0] + 2)
    ax.set_ylim(-2, map_size[1] + 2)
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.set_title(title, fontsize=12, fontweight='bold')
    ax.legend(loc='upper left', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.set_facecolor(COLORS['background'])

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    print(f"  Map saved: {output_path}")


def generate_comparison_visualization(config: Dict, uav_result: Dict, dog_result: Dict,
                                     uav_simplified: Dict, dog_simplified: Dict,
                                     output_path: str):
    map_size = (config['map_size']['x'], config['map_size']['y'])
    uav_config = config['uav']
    dog_config = config['dog']
    uav_obstacles = uav_config['obstacles']
    dog_obstacles = dog_config['obstacles']

    fig, axes = plt.subplots(1, 2, figsize=(20, 9))

    ax_uav = axes[0]
    ax_dog = axes[1]

    for obs in uav_obstacles:
        draw_obstacle(ax_uav, obs)
    for obs in dog_obstacles:
        draw_obstacle(ax_dog, obs)

    ax_uav.scatter(uav_config['start']['x'], uav_config['start']['y'],
                    c=COLORS['start'], s=300, marker='s', zorder=5,
                    label='Start', edgecolors='black', linewidths=2)
    ax_uav.scatter(uav_config['goal']['x'], uav_config['goal']['y'],
                    c=COLORS['goal'], s=300, marker='*', zorder=5,
                    label='Goal', edgecolors='black', linewidths=2)

    ax_dog.scatter(dog_config['start']['x'], dog_config['start']['y'],
                    c=COLORS['start'], s=300, marker='s', zorder=5,
                    label='Start', edgecolors='black', linewidths=2)
    ax_dog.scatter(dog_config['goal']['x'], dog_config['goal']['y'],
                    c=COLORS['goal'], s=300, marker='*', zorder=5,
                    label='Goal', edgecolors='black', linewidths=2)

    if uav_result.get('result') and uav_result.get('metrics'):
        uav_points = uav_result['result']['points']
        ux = [p['x'] for p in uav_points]
        uy = [p['y'] for p in uav_points]
        ax_uav.plot(ux, uy, '-', color=COLORS['uav'], linewidth=3,
                   label=f'Path ({uav_result["metrics"]["path_length"]:.1f}m)')
        ax_uav.scatter(ux, uy, c=COLORS['uav'], s=30, alpha=0.7)
        uav_title = f"UAV View ({len(uav_obstacles)} obstacles)\n"
        uav_title += f"Path: {uav_result['metrics']['path_length']:.1f}m | "
        uav_title += f"Pts: {uav_result['metrics']['num_waypoints']} | "
        uav_title += f"Smooth: {uav_result['metrics']['smoothness']:.3f}"
    else:
        ax_uav.text(map_size[0]/2, map_size[1]/2, "No valid path found",
                   ha='center', va='center', fontsize=14, color='red')
        uav_title = f"UAV View ({len(uav_obstacles)} obstacles)\nNo path found"

    if dog_result.get('result') and dog_result.get('metrics'):
        dog_points = dog_result['result']['points']
        dx = [p['x'] for p in dog_points]
        dy = [p['y'] for p in dog_points]
        ax_dog.plot(dx, dy, '-', color=COLORS['dog'], linewidth=3,
                   label=f'Path ({dog_result["metrics"]["path_length"]:.1f}m)')
        ax_dog.scatter(dx, dy, c=COLORS['dog'], s=30, alpha=0.7)
        dog_title = f"Dog View ({len(dog_obstacles)} obstacles)\n"
        dog_title += f"Path: {dog_result['metrics']['path_length']:.1f}m | "
        dog_title += f"Pts: {dog_result['metrics']['num_waypoints']} | "
        dog_title += f"Smooth: {dog_result['metrics']['smoothness']:.3f}"
    else:
        ax_dog.text(map_size[0]/2, map_size[1]/2, "No valid path found",
                   ha='center', va='center', fontsize=14, color='red')
        dog_title = f"Dog View ({len(dog_obstacles)} obstacles)\nNo path found"

    ax_uav.set_xlim(-2, map_size[0] + 2)
    ax_uav.set_ylim(-2, map_size[1] + 2)
    ax_uav.set_xlabel('X (meters)', fontsize=12)
    ax_uav.set_ylabel('Y (meters)', fontsize=12)
    ax_uav.set_title(uav_title, fontsize=12, fontweight='bold', color=COLORS['uav'])
    ax_uav.legend(loc='upper left', fontsize=10)
    ax_uav.grid(True, alpha=0.3)
    ax_uav.set_aspect('equal')
    ax_uav.set_facecolor(COLORS['background'])

    ax_dog.set_xlim(-2, map_size[0] + 2)
    ax_dog.set_ylim(-2, map_size[1] + 2)
    ax_dog.set_xlabel('X (meters)', fontsize=12)
    ax_dog.set_ylabel('Y (meters)', fontsize=12)
    ax_dog.set_title(dog_title, fontsize=12, fontweight='bold', color=COLORS['dog'])
    ax_dog.legend(loc='upper left', fontsize=10)
    ax_dog.grid(True, alpha=0.3)
    ax_dog.set_aspect('equal')
    ax_dog.set_facecolor(COLORS['background'])

    plt.suptitle(f"{config['name']}\nUAV vs Dog Path Comparison",
                fontsize=14, fontweight='bold', y=1.02)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    print(f"  Comparison saved: {output_path}")


def generate_summary_report(all_results: Dict, output_path: str):
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write("=" * 80 + "\n")
        f.write("UAV vs Dog Path Comparison Report\n")
        f.write("=" * 80 + "\n\n")

        for scenario_name, data in all_results.items():
            f.write(f"\n{'='*70}\n")
            f.write(f"Scenario: {scenario_name}\n")
            f.write(f"{'='*70}\n\n")

            f.write("-" * 50 + "\n")
            f.write("UAV Path Evaluation\n")
            f.write("-" * 50 + "\n")
            if data['uav']:
                m = data['uav']['metrics']
                s = data['uav'].get('simplified', {})
                f.write(f"  Obstacles: {len(data['uav_config']['obstacles'])}\n")
                f.write(f"  Start: ({data['uav_config']['start']['x']}, {data['uav_config']['start']['y']})\n")
                f.write(f"  Goal: ({data['uav_config']['goal']['x']}, {data['uav_config']['goal']['y']})\n")
                f.write(f"  Path Length: {m['path_length']:.2f} m\n")
                f.write(f"  Waypoints: {m['num_waypoints']}\n")
                f.write(f"  Smoothness: {m['smoothness']:.3f}\n")
                f.write(f"  Computation Time: {m['computation_time_ms']:.1f} ms\n")
                if s.get('simplified'):
                    f.write(f"  Simplified Points: {len(s['simplified']['points'])}\n")

            f.write("\n" + "-" * 50 + "\n")
            f.write("Dog Path Evaluation\n")
            f.write("-" * 50 + "\n")
            if data['dog']:
                m = data['dog']['metrics']
                s = data['dog'].get('simplified', {})
                f.write(f"  Obstacles: {len(data['dog_config']['obstacles'])}\n")
                f.write(f"  Start: ({data['dog_config']['start']['x']}, {data['dog_config']['start']['y']})\n")
                f.write(f"  Goal: ({data['dog_config']['goal']['x']}, {data['dog_config']['goal']['y']})\n")
                f.write(f"  Path Length: {m['path_length']:.2f} m\n")
                f.write(f"  Waypoints: {m['num_waypoints']}\n")
                f.write(f"  Smoothness: {m['smoothness']:.3f}\n")
                f.write(f"  Computation Time: {m['computation_time_ms']:.1f} ms\n")
                if s.get('simplified'):
                    f.write(f"  Simplified Points: {len(s['simplified']['points'])}\n")

            f.write("\n" + "-" * 50 + "\n")
            f.write("Comparison Summary\n")
            f.write("-" * 50 + "\n")
            if data['uav'] and data['dog']:
                uav_m = data['uav']['metrics']
                dog_m = data['dog']['metrics']
                uav_obs = len(data['uav_config']['obstacles'])
                dog_obs = len(data['dog_config']['obstacles'])

                f.write(f"  UAV obstacles: {uav_obs}\n")
                f.write(f"  Dog obstacles: {dog_obs}\n")
                f.write(f"  Dog cannot see: {dog_obs - uav_obs} hidden passages\n\n")

                length_diff = uav_m['path_length'] - dog_m['path_length']

                f.write(f"  UAV Path Length: {uav_m['path_length']:.2f} m\n")
                f.write(f"  Dog Path Length: {dog_m['path_length']:.2f} m\n")

                if length_diff < 0:
                    f.write(f"  Difference: {-length_diff:.2f} m ({100*(-length_diff)/dog_m['path_length']:.1f}% shorter)\n\n")
                    f.write("  => UAV path is SHORTER (uses hidden passages)\n")
                else:
                    f.write(f"  Difference: {length_diff:.2f} m ({100*length_diff/uav_m['path_length']:.1f}% longer)\n\n")
                    f.write("  => Dog path is SHORTER\n")

                if uav_m['smoothness'] > dog_m['smoothness']:
                    f.write("  => UAV path is SMOOTHER\n")
                else:
                    f.write("  => Dog path is SMOOTHER\n")

                if uav_m['computation_time_ms'] < dog_m['computation_time_ms']:
                    f.write("  => UAV is FASTER\n")
                else:
                    f.write("  => Dog is FASTER\n")

            f.write("\n")

    print(f"\nSummary report saved: {output_path}")


def main():
    base_dir = os.path.dirname(__file__)

    scenarios = [
        ("scenario_a_simple", os.path.join(base_dir, "config/scenarios/scenario_a_simple.json")),
        ("scenario_b_maze", os.path.join(base_dir, "config/scenarios/scenario_b_maze.json")),
        ("scenario_c_complex", os.path.join(base_dir, "config/scenarios/scenario_c_complex.json"))
    ]

    print("=" * 80)
    print("UAV vs Dog Path Comparison")
    print("Dog cannot see hidden passages that UAV can fly over")
    print("=" * 80)

    all_results = {}

    for scenario_id, scenario_path in scenarios:
        config = load_scenario(scenario_path)
        scenario_key = config['name'].split(':')[0].strip()

        print(f"\n{'='*70}")
        print(f"Scenario: {config['name']}")
        print(f"{'='*70}")

        uav_config = config['uav']
        dog_config = config['dog']

        uav_start = (uav_config['start']['x'], uav_config['start']['y'])
        uav_goal = (uav_config['goal']['x'], uav_config['goal']['y'])
        dog_start = (dog_config['start']['x'], dog_config['start']['y'])
        dog_goal = (dog_config['goal']['x'], dog_config['goal']['y'])

        map_size = (config['map_size']['x'], config['map_size']['y'])

        uav_result = run_path_planning(
            uav_config['obstacles'], map_size, uav_start, uav_goal, "UAV"
        )
        dog_result = run_path_planning(
            dog_config['obstacles'], map_size, dog_start, dog_goal, "Dog"
        )

        uav_simplified = run_simplification(uav_result, "UAV")
        dog_simplified = run_simplification(dog_result, "Dog")

        uav_result['simplified'] = uav_simplified
        dog_result['simplified'] = dog_simplified

        os.makedirs("output", exist_ok=True)
        os.makedirs(f"output/{scenario_id}", exist_ok=True)
        os.makedirs(f"config/scenarios/{scenario_id}", exist_ok=True)

        config_dir = os.path.dirname(scenario_path)
        generate_map_image(config, uav_config['obstacles'],
                          f"{config_dir}/{scenario_id}/map_uav.png", "uav")
        generate_map_image(config, dog_config['obstacles'],
                          f"{config_dir}/{scenario_id}/map_dog.png", "dog")

        generate_comparison_visualization(
            config, uav_result, dog_result,
            uav_simplified, dog_simplified,
            f"output/{scenario_id}/{scenario_id}_comparison.png"
        )

        all_results[scenario_key] = {
            'uav': uav_result,
            'dog': dog_result,
            'uav_config': uav_config,
            'dog_config': dog_config
        }

    print(f"\n{'='*80}")
    print("Final Summary")
    print(f"{'='*80}")

    for scenario_name, data in all_results.items():
        print(f"\n{scenario_name}:")
        if data['uav'] and data['dog']:
            uav_obs = len(data['uav_config']['obstacles'])
            dog_obs = len(data['dog_config']['obstacles'])
            uav_len = data['uav']['metrics']['path_length']
            dog_len = data['dog']['metrics']['path_length']

            print(f"  UAV obstacles: {uav_obs}, Path: {uav_len:.2f}m")
            print(f"  Dog obstacles: {dog_obs}, Path: {dog_len:.2f}m")
            print(f"  Hidden passages: {dog_obs - uav_obs}")

            if uav_len < dog_len:
                diff = dog_len - uav_len
                pct = 100 * diff / dog_len
                print(f"  => UAV path is {diff:.2f}m shorter ({pct:.1f}% reduction)")

    generate_summary_report(all_results, "output/comparison_summary.txt")

    print(f"\n{'='*80}")
    print("Output Files:")
    print("Config maps:")
    print("  config/scenarios/scenario_a_simple/map_uav.png")
    print("  config/scenarios/scenario_a_simple/map_dog.png")
    print("  config/scenarios/scenario_b_maze/map_uav.png")
    print("  config/scenarios/scenario_b_maze/map_dog.png")
    print("  config/scenarios/scenario_c_complex/map_uav.png")
    print("  config/scenarios/scenario_c_complex/map_dog.png")
    print("\nComparison:")
    print("  output/scenario_a_simple/scenario_a_simple_comparison.png")
    print("  output/scenario_b_maze/scenario_b_maze_comparison.png")
    print("  output/scenario_c_complex/scenario_c_complex_comparison.png")
    print("  output/comparison_summary.txt")
    print(f"{'='*80}")


if __name__ == "__main__":
    main()
