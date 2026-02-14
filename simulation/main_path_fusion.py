#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Fusion Main Program

Path fusion implementation for UAV and Dog cooperation.
Dog perception radius: R = min(W, H) / 2
"""

import sys
import os
import json
import time
import math
import random
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

sys.path.insert(0, os.path.dirname(__file__))

from map_generator import Obstacle2D
from algorithm.a_star import AStar
from algorithm.perception_radius import (
    generate_dog_map,
    calculate_perception_boundary,
    Point
)
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
    'boundary': '#9932CC',
    'final': '#00CC00',
    'hidden': '#AAAAAA',
    'mosaic': '#CCCCCC'
}


@dataclass
class PathResult:
    status: str
    points: List[Dict]
    metrics: Dict
    boundary_point: Optional[Dict] = None
    reason: str = ""


def load_scenario(filepath: str) -> Dict:
    with open(filepath, 'r') as f:
        return json.load(f)


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


def draw_mosaic_region(ax, center: Tuple[float, float], radius: float):
    """Draw mosaic/hatched region to represent invisible areas"""
    x, y = center
    num_wedges = 24
    
    for i in range(num_wedges):
        angle1 = 2 * math.pi * i / num_wedges
        angle2 = 2 * math.pi * (i + 1) / num_wedges
        
        vertices = [(x, y)]
        for angle in [angle1, angle2]:
            px = x + radius * math.cos(angle)
            py = y + radius * math.sin(angle)
            vertices.append((px, py))
        vertices.append((x, y))
        
        alpha = 0.1 + random.random() * 0.08
        gray_value = 0.6 + random.random() * 0.2
        color = str(gray_value)
        
        polygon = plt.Polygon(vertices, facecolor=color, 
                             edgecolor='none', alpha=alpha, zorder=0)
        ax.add_patch(polygon)
    
    circle = plt.Circle((x, y), radius, fill=False, color=COLORS['boundary'],
                        linewidth=2.5, linestyle='--', zorder=2)
    ax.add_patch(circle)


def run_path_planning(obstacles: List[Dict], map_size: Tuple,
                      start: Tuple, goal: Tuple, agent_type: str) -> PathResult:
    """Run path planning"""
    print(f"\n  [{agent_type}] Start: {start}, Goal: {goal}")
    print(f"  [{agent_type}] Obstacles: {len(obstacles)}")

    obs_objects = [Obstacle2D.from_dict(obs) for obs in obstacles]
    planner = AStar(obs_objects, map_size)

    start_time = time.time()
    result = planner.plan(start, goal, heuristic_method="euclidean")
    planning_time = time.time() - start_time

    if result:
        theoretical_dist = math.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2)
        metrics = PathMetrics.calculate_all(result, start_time, time.time(),
                                           theoretical_distance=theoretical_dist,
                                           map_size=map_size)
        print(f"  [{agent_type}] Path: {metrics['path_length']:.2f}m, "
              f"{metrics['num_waypoints']} pts, turns: {metrics['num_turns']}, "
              f"smoothness: {metrics['smoothness']:.3f}, time: {metrics['computation_time_ms']:.1f}ms")
        return PathResult(status="complete", points=result['points'], metrics=metrics)
    else:
        print(f"  [{agent_type}] Planning failed!")
        return PathResult(status="failed", points=[], metrics={})


def run_boundary_planning(obstacles: List[Dict], map_size: Tuple,
                         start: Tuple, boundary: Tuple, agent_type: str) -> PathResult:
    """Plan shortest path from start to boundary point (not straight line)"""
    print(f"\n  [{agent_type}] Planning to boundary: {boundary}")

    obs_objects = [Obstacle2D.from_dict(obs) for obs in obstacles]
    planner = AStar(obs_objects, map_size)

    start_time = time.time()
    result = planner.plan(start, boundary, heuristic_method="euclidean")
    planning_time = time.time() - start_time

    if result:
        theoretical_dist = math.sqrt((boundary[0]-start[0])**2 + (boundary[1]-start[1])**2)
        metrics = PathMetrics.calculate_all(result, start_time, time.time(),
                                           theoretical_distance=theoretical_dist,
                                           map_size=map_size)
        print(f"  [{agent_type}] Boundary path: {metrics['path_length']:.2f}m, "
              f"turns: {metrics['num_turns']}")
        return PathResult(status="partial", points=result['points'], metrics=metrics,
                         boundary_point={'x': boundary[0], 'y': boundary[1]},
                         reason="Goal outside perception radius")
    else:
        print(f"  [{agent_type}] Boundary planning failed!")
        return PathResult(status="failed", points=[], metrics={})


def run_simplification(result: PathResult, agent_type: str) -> PathResult:
    """Run path simplification"""
    if not result.points:
        return result

    points = result.points
    simplified = rdp_simplify([(p['x'], p['y']) for p in points], epsilon=1.5)

    original_length = sum(((points[i]['x']-points[i-1]['x'])**2 +
                          (points[i]['y']-points[i-1]['y'])**2)**0.5
                         for i in range(1, len(points)))
    simplified_length = sum(((simplified[i][0]-simplified[i-1][0])**2 +
                           (simplified[i][1]-simplified[i-1][1])**2)**0.5
                           for i in range(1, len(simplified)))

    print(f"  [{agent_type}] Simplified: {len(points)} -> {len(simplified)} pts "
          f"({100*(len(points)-len(simplified))/len(points):.1f}% reduction)")

    result.points = [{'x': p[0], 'y': p[1]} for p in simplified]
    result.metrics['simplified_length'] = round(simplified_length, 2)
    result.metrics['simplified_points'] = len(simplified)

    return result


def fuse_paths(uav_result: PathResult, dog_result: PathResult) -> Tuple[PathResult, Dict]:
    """Fuse two paths and select optimal"""
    decision = {
        'uav_status': uav_result.status,
        'dog_status': dog_result.status,
        'selected': None,
        'reason': [],
        'metrics_comparison': {}
    }

    if uav_result.status != 'complete':
        decision['selected'] = 'dog'
        decision['reason'].append("UAV planning failed, use Dog path")
        return dog_result, decision

    if dog_result.status == 'complete':
        uav_metrics = uav_result.metrics
        dog_metrics = dog_result.metrics

        decision['metrics_comparison'] = {
            'length_uav': uav_metrics.get('path_length'),
            'length_dog': dog_metrics.get('path_length'),
            'turns_uav': uav_metrics.get('num_turns'),
            'turns_dog': dog_metrics.get('num_turns'),
            'smoothness_uav': uav_metrics.get('smoothness'),
            'smoothness_dog': dog_metrics.get('smoothness')
        }

        length_uav = uav_metrics.get('path_length', float('inf'))
        length_dog = dog_metrics.get('path_length', float('inf'))

        if length_dog > 0:
            length_diff_pct = (length_uav - length_dog) / length_dog * 100

            if length_diff_pct < -5:
                decision['selected'] = 'uav'
                decision['reason'].append(f"UAV path shorter by {abs(length_diff_pct):.1f}%")
            elif length_diff_pct > 5:
                decision['selected'] = 'dog'
                decision['reason'].append(f"Dog path shorter by {length_diff_pct:.1f}%")
            else:
                turns_uav = uav_metrics.get('num_turns', 0)
                turns_dog = dog_metrics.get('num_turns', 0)

                if turns_uav < turns_dog:
                    decision['selected'] = 'uav'
                    decision['reason'].append(f"UAV fewer turns ({turns_uav} vs {turns_dog})")
                elif turns_dog < turns_uav:
                    decision['selected'] = 'dog'
                    decision['reason'].append(f"Dog fewer turns ({turns_dog} vs {turns_uav})")
                else:
                    smooth_uav = uav_metrics.get('smoothness', 0)
                    smooth_dog = dog_metrics.get('smoothness', 0)

                    if smooth_uav > smooth_dog:
                        decision['selected'] = 'uav'
                        decision['reason'].append(f"UAV smoother ({smooth_uav:.3f} vs {smooth_dog:.3f})")
                    else:
                        decision['selected'] = 'dog'
                        decision['reason'].append(f"Dog smoother ({smooth_dog:.3f} vs {smooth_uav:.3f})")

    elif dog_result.status == 'partial':
        decision['selected'] = 'uav'
        decision['reason'].append("Dog can only plan to boundary, use UAV full path")
        decision['reason'].append(dog_result.reason)

    else:
        decision['selected'] = 'uav'
        decision['reason'].append("Dog planning failed, use UAV path")

    if not decision['reason']:
        decision['reason'].append("Use UAV path (default)")
        decision['selected'] = 'uav'

    final_result = uav_result if decision['selected'] == 'uav' else dog_result
    final_result.status = 'final'

    return final_result, decision


def visualize_comparison(config: Dict, uav_result: PathResult, dog_result: PathResult,
                        final_result: PathResult, decision: Dict, output_path: str):
    """Visualize path comparison"""
    map_size = (config['map_size']['x'], config['map_size']['y'])
    start = (config['start']['x'], config['start']['y'])
    goal = (config['goal']['x'], config['goal']['y'])
    radius = config.get('perception_radius', 30)

    fig, axes = plt.subplots(1, 3, figsize=(24, 8))

    full_obstacles = config.get('obstacles', [])
    dog_config = generate_dog_map(config)
    dog_obstacles = dog_config.get('obstacles', [])

    for ax in axes:
        for obs in full_obstacles:
            draw_obstacle(ax, obs)
        ax.scatter(start[0], start[1], c=COLORS['start'], s=300, marker='s', zorder=5,
                   label='Start', edgecolors='black', linewidths=2)
        ax.scatter(goal[0], goal[1], c=COLORS['goal'], s=300, marker='*', zorder=5,
                   label='Goal', edgecolors='black', linewidths=2)

    circle = plt.Circle(start, radius, fill=False, color=COLORS['boundary'],
                        linewidth=2, linestyle='--', label=f'R={radius}')
    axes[0].add_patch(circle)

    axes[0].set_xlim(-2, map_size[0] + 2)
    axes[0].set_ylim(-2, map_size[1] + 2)
    axes[0].set_xlabel('X (meters)', fontsize=12)
    axes[0].set_ylabel('Y (meters)', fontsize=12)
    axes[0].set_title(f"Full Map (UAV View)\n{len(full_obstacles)} obstacles", 
                      fontsize=12, fontweight='bold')
    axes[0].legend(loc='upper left', fontsize=10)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_aspect('equal')
    axes[0].set_facecolor(COLORS['background'])

    for obs in dog_obstacles:
        draw_obstacle(axes[1], obs)
    
    draw_mosaic_region(axes[1], start, radius)

    axes[1].set_xlim(-2, map_size[0] + 2)
    axes[1].set_ylim(-2, map_size[1] + 2)
    axes[1].set_xlabel('X (meters)', fontsize=12)
    axes[1].set_ylabel('Y (meters)', fontsize=12)
    axes[1].set_title(f"Dog View (R={radius})\n{len(dog_obstacles)} obstacles visible\nGray = Unknown region", 
                      fontsize=11, fontweight='bold')
    axes[1].legend(loc='upper left', fontsize=10)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_aspect('equal')
    axes[1].set_facecolor(COLORS['background'])

    if uav_result.points:
        ux = [p['x'] for p in uav_result.points]
        uy = [p['y'] for p in uav_result.points]
        axes[2].plot(ux, uy, '-', color=COLORS['uav'], linewidth=3, alpha=0.7,
                    label=f'UAV Path ({uav_result.metrics.get("path_length", "N/A")}m)')

    if dog_result.points:
        dx = [p['x'] for p in dog_result.points]
        dy = [p['y'] for p in dog_result.points]
        label = f'Dog Path ({dog_result.metrics.get("path_length", "N/A")}m)'
        if dog_result.status == 'partial':
            label += ' (to boundary)'
        axes[2].plot(dx, dy, '-', color=COLORS['dog'], linewidth=3, alpha=0.7, label=label)
        
        if dog_result.boundary_point:
            axes[2].scatter(dog_result.boundary_point['x'], dog_result.boundary_point['y'],
                           c=COLORS['boundary'], s=200, marker='o', zorder=6,
                           edgecolors='black', linewidths=2, label='Boundary Point')

    if final_result.points:
        fx = [p['x'] for p in final_result.points]
        fy = [p['y'] for p in final_result.points]
        axes[2].plot(fx, fy, '--', color=COLORS['final'], linewidth=4,
                    label=f'Final ({final_result.metrics.get("path_length", "N/A")}m)')

    axes[2].set_xlim(-2, map_size[0] + 2)
    axes[2].set_ylim(-2, map_size[1] + 2)
    axes[2].set_xlabel('X (meters)', fontsize=12)
    axes[2].set_ylabel('Y (meters)', fontsize=12)

    reason_text = ' | '.join(decision.get('reason', []))
    axes[2].set_title(f"Path Comparison\nSelected: {decision.get('selected', 'N/A').upper()} | {reason_text}",
                      fontsize=11, fontweight='bold')
    axes[2].legend(loc='upper left', fontsize=10)
    axes[2].grid(True, alpha=0.3)
    axes[2].set_aspect('equal')
    axes[2].set_facecolor(COLORS['background'])

    plt.suptitle(f"{config['name']}\nUAV vs Dog Path Fusion", fontsize=14, fontweight='bold', y=1.02)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    print(f"  Visualization saved: {output_path}")


def visualize_metrics(uav_metrics: Dict, dog_metrics: Dict, decision: Dict,
                    output_path: str):
    """Visualize metrics comparison table"""
    def fmt(val, fmt_str='.3f'):
        if val is None or val == 'N/A':
            return 'N/A'
        try:
            return f"{val:{fmt_str}}"
        except (ValueError, TypeError):
            return str(val)

    fig, ax = plt.subplots(figsize=(12, 6))
    ax.axis('off')

    metrics_data = [
        ['Metric', 'UAV', 'Dog', 'Difference'],
        ['Path Length (m)', fmt(uav_metrics.get('path_length'), '.2f'),
         fmt(dog_metrics.get('path_length'), '.2f'),
         fmt(decision['metrics_comparison'].get('length_diff'), '.2f')],
        ['Waypoints', str(uav_metrics.get('num_waypoints', 'N/A')),
         str(dog_metrics.get('num_waypoints', 'N/A')), '-'],
        ['Turns', str(uav_metrics.get('num_turns', 'N/A')),
         str(dog_metrics.get('num_turns', 'N/A')),
         str(decision['metrics_comparison'].get('turn_diff', 'N/A'))],
        ['Smoothness', fmt(uav_metrics.get('smoothness')),
         fmt(dog_metrics.get('smoothness')),
         fmt(decision['metrics_comparison'].get('smoothness_diff'))],
        ['Computation Time (ms)', fmt(uav_metrics.get('computation_time_ms'), '.1f'),
         fmt(dog_metrics.get('computation_time_ms'), '.1f'),
         fmt(decision['metrics_comparison'].get('time_diff'), '.1f')],
        ['Optimality Ratio', fmt(uav_metrics.get('optimality_ratio')),
         fmt(dog_metrics.get('optimality_ratio')), '-']
    ]

    table = ax.table(cellText=metrics_data[1:], colLabels=metrics_data[0],
                     cellLoc='center', loc='center',
                     colWidths=[0.25, 0.2, 0.2, 0.2])
    table.auto_set_font_size(False)
    table.set_fontsize(12)
    table.scale(1.2, 2)

    for i in range(len(metrics_data[0])):
        table[(0, i)].set_facecolor('#4472C4')
        table[(0, i)].set_text_props(color='white', fontweight='bold')

    for i in range(1, len(metrics_data)):
        if decision.get('selected') == 'uav' and i in [1, 3, 4]:
            table[(i, 1)].set_facecolor('#90EE90')
        elif decision.get('selected') == 'dog' and i in [1, 3, 4]:
            table[(i, 2)].set_facecolor('#90EE90')

    ax.set_title("Path Metrics Comparison\n" +
                 f"Selected: {decision.get('selected', 'N/A').upper()} | " +
                 " | ".join(decision.get('reason', [])),
                 fontsize=14, fontweight='bold', pad=20)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    print(f"  Metrics table saved: {output_path}")


def generate_summary_report(config: Dict, uav_result: PathResult, dog_result: PathResult,
                           final_result: PathResult, decision: Dict, output_path: str):
    """Generate summary report"""
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write("=" * 80 + "\n")
        f.write("UAV vs Dog Path Fusion Report\n")
        f.write("=" * 80 + "\n\n")

        f.write(f"Scenario: {config['name']}\n")
        f.write(f"Map Size: {config['map_size']['x']} x {config['map_size']['y']}\n")
        f.write(f"Perception Radius: {config.get('perception_radius', 'N/A')}m\n")
        f.write(f"Start: ({config['start']['x']}, {config['start']['y']})\n")
        f.write(f"Goal: ({config['goal']['x']}, {config['goal']['y']})\n\n")

        f.write("-" * 50 + "\n")
        f.write("UAV Path (Full Map)\n")
        f.write("-" * 50 + "\n")
        if uav_result.metrics:
            for key, value in uav_result.metrics.items():
                f.write(f"  {key}: {value}\n")

        f.write("\n" + "-" * 50 + "\n")
        f.write("Dog Path (Partial Map)\n")
        f.write("-" * 50 + "\n")
        f.write(f"  Status: {dog_result.status}\n")
        if dog_result.boundary_point:
            f.write(f"  Boundary Point: ({dog_result.boundary_point['x']:.2f}, {dog_result.boundary_point['y']:.2f})\n")
        if dog_result.reason:
            f.write(f"  Reason: {dog_result.reason}\n")
        if dog_result.metrics:
            for key, value in dog_result.metrics.items():
                f.write(f"  {key}: {value}\n")

        f.write("\n" + "-" * 50 + "\n")
        f.write("Fusion Decision\n")
        f.write("-" * 50 + "\n")
        f.write(f"  Selected Path: {decision.get('selected', 'N/A').upper()}\n")
        f.write(f"  Reasons:\n")
        for reason in decision.get('reason', []):
            f.write(f"    - {reason}\n")

        if final_result.metrics:
            f.write(f"\n  Final Path Metrics:\n")
            for key, value in final_result.metrics.items():
                f.write(f"    {key}: {value}\n")

    print(f"Summary report saved: {output_path}")


def main():
    base_dir = os.path.dirname(__file__)

    scenarios = [
        ("scenario_a_simple", os.path.join(base_dir, "config/scenarios/scenario_a_simple.json")),
        ("scenario_b_maze", os.path.join(base_dir, "config/scenarios/scenario_b_maze.json")),
        ("scenario_c_complex", os.path.join(base_dir, "config/scenarios/scenario_c_complex.json"))
    ]

    print("=" * 80)
    print("UAV vs Dog Path Fusion")
    print("Dog perception radius: R = min(W, H) / 2")
    print("=" * 80)

    for scenario_id, scenario_path in scenarios:
        config = load_scenario(scenario_path)

        print(f"\n{'='*70}")
        print(f"Scenario: {config['name']}")
        print(f"{'='*70}")

        start = (config['start']['x'], config['start']['y'])
        goal = (config['goal']['x'], config['goal']['y'])
        map_size = (config['map_size']['x'], config['map_size']['y'])
        radius = config.get('perception_radius', 30)

        print(f"  Map: {map_size[0]}x{map_size[1]}, R={radius}")
        print(f"  Start: {start}, Goal: {goal}")

        dog_config = generate_dog_map(config)
        print(f"  UAV obstacles: {len(config.get('obstacles', []))}")
        print(f"  Dog visible obstacles: {len(dog_config.get('obstacles', []))}")

        uav_result = run_path_planning(config.get('obstacles', []), map_size,
                                       start, goal, "UAV")
        uav_result = run_simplification(uav_result, "UAV")

        start_point = Point(start[0], start[1])
        goal_point = Point(goal[0], goal[1])
        boundary_pt, goal_in_range = calculate_perception_boundary(start_point, goal_point, radius)

        if goal_in_range:
            dog_result = run_path_planning(dog_config.get('obstacles', []), map_size,
                                          start, goal, "Dog")
            dog_result = run_simplification(dog_result, "Dog")
        elif boundary_pt:
            dog_result = run_boundary_planning(
                dog_config.get('obstacles', []), map_size,
                start, (boundary_pt.x, boundary_pt.y), "Dog"
            )
            dog_result = run_simplification(dog_result, "Dog")
        else:
            dog_result = PathResult(status="failed", points=[], metrics={},
                                   reason="Cannot calculate boundary point")

        os.makedirs("output", exist_ok=True)
        os.makedirs(f"output/{scenario_id}", exist_ok=True)

        final_result, decision = fuse_paths(uav_result, dog_result)

        print(f"\n{'='*50}")
        print(f"Final Decision: {decision.get('selected', 'N/A').upper()}")
        for reason in decision.get('reason', []):
            print(f"  - {reason}")

        visualize_comparison(config, uav_result, dog_result, final_result, decision,
                           f"output/{scenario_id}/{scenario_id}_fusion.png")

        if uav_result.metrics and dog_result.metrics:
            visualize_metrics(uav_result.metrics, dog_result.metrics, decision,
                             f"output/{scenario_id}/{scenario_id}_metrics.png")

        generate_summary_report(config, uav_result, dog_result, final_result, decision,
                               f"output/{scenario_id}/{scenario_id}_report.txt")

    print(f"\n{'='*80}")
    print("All scenarios completed!")
    print("=" * 80)


if __name__ == "__main__":
    main()
