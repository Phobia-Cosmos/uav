#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ground-Air semantic fusion simulation.

核心机制：
1. 无人机先在“透视地图”上做全局规划；
2. 机器狗沿路径前进；
3. 当视域圆与未知特殊地形/障碍物相切时立即暂停；
4. 上传真实几何与语义；
5. 无人机修正融合地图并重新规划；
6. 机器狗继续沿新路径前进。
"""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from algorithm.a_star import AStar
from evaluation.metrics import PathMetrics
from semantic_maps import (
    CRAWL_TYPES,
    SWAMP_TYPES,
    build_discovered_semantic_map,
    build_fused_planning_obstacles,
    build_fused_visual_obstacles,
    build_semantic_packet,
    build_uav_perspective_obstacles,
    clip_path_to_trigger,
    find_next_trigger,
    get_physical_obstacles,
    merge_trace,
    obstacle_semantic_type,
    obstacle_visible_from_position,
    simplify_path_with_line_of_sight,
)
from map_generator import Obstacle2D


COLORS = {
    'background': '#FAF8F2',
    'building': '#8D6E63',
    'wall': '#6D4C41',
    'debris': '#78909C',
    'swamp_fill': '#4DB6AC',
    'swamp_edge': '#00695C',
    'crawl_fill': '#E0E0E0',
    'crawl_edge': '#616161',
    'start': '#2E7D32',
    'goal': '#C62828',
    'initial_path': '#1565C0',
    'replanned_path': '#8E24AA',
    'final_path': '#EF6C00',
    'executed': '#00A86B',
    'circle': '#AB47BC',
    'event': '#D81B60',
}


@dataclass
class PlanSnapshot:
    label: str
    start: Dict[str, float]
    goal: Dict[str, float]
    points: List[Dict[str, float]]
    metrics: Dict[str, float]
    obstacle_count: int


@dataclass
class TriggerEvent:
    index: int
    stop_point: Dict[str, float]
    triggered_obstacle_id: str
    uploaded_packets: List[Dict]
    visible_ids: List[str]
    source_plan: str


@dataclass
class ScenarioResult:
    config: Dict
    initial_plan: Optional[PlanSnapshot]
    plans: List[PlanSnapshot] = field(default_factory=list)
    events: List[TriggerEvent] = field(default_factory=list)
    executed_trace: List[Dict[str, float]] = field(default_factory=list)
    discovered_ids: Set[str] = field(default_factory=set)
    success: bool = False
    failure_reason: str = ""

    @property
    def final_plan(self) -> Optional[PlanSnapshot]:
        return self.plans[-1] if self.plans else None


def load_scenario(filepath: str) -> Dict:
    with open(filepath, 'r', encoding='utf-8') as file:
        return json.load(file)


def plan_route(start: Tuple[float, float],
               goal: Tuple[float, float],
               obstacles: List[Dict],
               map_size: Tuple[float, float],
               label: str) -> Optional[PlanSnapshot]:
    planner = AStar([Obstacle2D.from_dict(obstacle) for obstacle in obstacles], map_size)
    start_time = time.time()
    result = planner.plan(start, goal, heuristic_method="euclidean")
    end_time = time.time()

    if not result:
        return None

    result["points"] = simplify_path_with_line_of_sight(result["points"], obstacles)

    theoretical_distance = math.sqrt((goal[0] - start[0]) ** 2 + (goal[1] - start[1]) ** 2)
    metrics = PathMetrics.calculate_all(
        result,
        start_time,
        end_time,
        theoretical_distance=theoretical_distance,
        map_size=map_size,
    )
    return PlanSnapshot(
        label=label,
        start={"x": start[0], "y": start[1]},
        goal={"x": goal[0], "y": goal[1]},
        points=result["points"],
        metrics=metrics,
        obstacle_count=len(obstacles),
    )


def point_distance(point_a: Sequence[float], point_b: Sequence[float]) -> float:
    return math.sqrt((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2)


def format_semantic_name(semantic_type: str) -> str:
    mapping = {
        "crawl_under_wall": "可钻行矮墙",
        "low_wall": "可钻行矮墙",
        "climbable_wall": "可翻越墙体",
        "swamp": "沼泽",
        "quicksand": "流沙",
        "water": "水面",
        "dead_end_wall": "死胡同墙体",
        "building": "建筑",
        "generic": "普通障碍物",
    }
    return mapping.get(semantic_type, semantic_type)


def obstacle_style(obstacle: Dict, view: str, discovered: bool = False) -> Optional[Dict]:
    semantic_type = obstacle_semantic_type(obstacle)

    if view == 'uav' and semantic_type in SWAMP_TYPES:
        return None
    if view == 'fused' and semantic_type in SWAMP_TYPES and not discovered:
        return None

    if semantic_type in CRAWL_TYPES:
        if view == 'uav' or (view == 'fused' and not discovered):
            return {
                'facecolor': '#9E9E9E',
                'edgecolor': '#424242',
                'alpha': 0.9,
                'linewidth': 2,
                'linestyle': '-',
                'hatch': None,
            }
        return {
            'facecolor': COLORS['crawl_fill'],
            'edgecolor': COLORS['crawl_edge'],
            'alpha': 0.55,
            'linewidth': 2,
            'linestyle': '--',
            'hatch': '///',
        }

    if semantic_type in SWAMP_TYPES:
        return {
            'facecolor': COLORS['swamp_fill'],
            'edgecolor': COLORS['swamp_edge'],
            'alpha': 0.45,
            'linewidth': 2,
            'linestyle': '-',
            'hatch': 'xx',
        }

    if obstacle.get('type') == 'circle':
        return {
            'facecolor': COLORS['debris'],
            'edgecolor': '#37474F',
            'alpha': 0.8,
            'linewidth': 1.8,
            'linestyle': '-',
            'hatch': None,
        }

    return {
        'facecolor': COLORS['wall'] if obstacle.get('type') == 'wall' else COLORS['building'],
        'edgecolor': '#2E2723',
        'alpha': 0.85,
        'linewidth': 2,
        'linestyle': '-',
        'hatch': None,
    }


def draw_obstacle(ax, obstacle: Dict, view: str = 'physical', discovered: bool = False):
    style = obstacle_style(obstacle, view, discovered=discovered)
    if style is None:
        return

    obstacle_type = obstacle.get('type', 'rectangle')
    if obstacle_type == 'rectangle':
        x_pos = obstacle['position']['x']
        y_pos = obstacle['position']['y']
        width = obstacle['size']['width']
        height = obstacle['size']['height']
        patch = plt.Rectangle(
            (x_pos - width / 2, y_pos - height / 2),
            width,
            height,
            **style,
        )
        ax.add_patch(patch)
        return

    if obstacle_type == 'circle':
        patch = plt.Circle(
            (obstacle['position']['x'], obstacle['position']['y']),
            obstacle['size']['radius'],
            **style,
        )
        ax.add_patch(patch)
        return

    if obstacle_type == 'wall':
        x1 = obstacle['start_point']['x']
        y1 = obstacle['start_point']['y']
        x2 = obstacle['end_point']['x']
        y2 = obstacle['end_point']['y']
        width = obstacle.get('width', 1)
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx * dx + dy * dy)
        if length < 1e-9:
            return
        nx = -dy / length * width / 2
        ny = dx / length * width / 2
        patch = plt.Polygon(
            [(x1 + nx, y1 + ny), (x2 + nx, y2 + ny), (x2 - nx, y2 - ny), (x1 - nx, y1 - ny)],
            **style,
        )
        ax.add_patch(patch)
        return

    if obstacle_type == 'polygon':
        patch = plt.Polygon(
            [(vertex['x'], vertex['y']) for vertex in obstacle['vertices']],
            closed=True,
            **style,
        )
        ax.add_patch(patch)


def draw_path(ax,
              points: List[Dict[str, float]],
              color: str,
              label: str,
              linestyle: str = '-',
              linewidth: float = 2.5,
              alpha: float = 0.95):
    if len(points) < 2:
        return
    xs = [point['x'] for point in points]
    ys = [point['y'] for point in points]
    ax.plot(xs, ys, linestyle=linestyle, color=color, linewidth=linewidth, alpha=alpha, label=label)


def decorate_axes(ax, title: str, map_size: Tuple[float, float], start: Tuple[float, float], goal: Tuple[float, float]):
    ax.scatter(start[0], start[1], c=COLORS['start'], s=180, marker='s', edgecolors='black', linewidths=1.5, zorder=6)
    ax.scatter(goal[0], goal[1], c=COLORS['goal'], s=220, marker='*', edgecolors='black', linewidths=1.2, zorder=6)
    ax.set_xlim(-2, map_size[0] + 2)
    ax.set_ylim(-2, map_size[1] + 2)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(title, fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.25)
    ax.set_aspect('equal')
    ax.set_facecolor(COLORS['background'])


def draw_event_overlays(ax, events: List[TriggerEvent], radius: float):
    for event in events:
        x_pos = event.stop_point['x']
        y_pos = event.stop_point['y']
        circle = plt.Circle((x_pos, y_pos), radius, fill=False, color=COLORS['circle'],
                            linestyle='--', linewidth=1.5, alpha=0.8)
        ax.add_patch(circle)
        ax.scatter(x_pos, y_pos, c=COLORS['event'], s=55, zorder=7)
        ax.text(x_pos + 0.8, y_pos + 0.8, f"E{event.index}", color=COLORS['event'], fontsize=9, weight='bold')


def simulate_scenario(config: Dict) -> ScenarioResult:
    map_size = (config['map_size']['x'], config['map_size']['y'])
    start = (config['start']['x'], config['start']['y'])
    goal = (config['goal']['x'], config['goal']['y'])
    radius = config.get('perception_radius', 12)

    result = ScenarioResult(config=config, initial_plan=None)
    current_position = start
    result.executed_trace.append({"x": start[0], "y": start[1]})

    physical_obstacles = get_physical_obstacles(config)
    special_count = sum(1 for obstacle in physical_obstacles if obstacle_semantic_type(obstacle) in (CRAWL_TYPES | SWAMP_TYPES))
    max_iterations = max(4, special_count + 4)

    for _ in range(max_iterations):
        planning_obstacles = build_fused_planning_obstacles(config, result.discovered_ids)
        plan_label = f"P_{len(result.plans)}"
        snapshot = plan_route(current_position, goal, planning_obstacles, map_size, plan_label)
        if snapshot is None:
            result.failure_reason = f"{plan_label} 规划失败"
            return result

        result.plans.append(snapshot)
        if result.initial_plan is None:
            result.initial_plan = snapshot

        candidate = find_next_trigger(snapshot.points, config, radius, result.discovered_ids)
        if candidate is None:
            merge_trace(result.executed_trace, snapshot.points)
            result.success = True
            return result

        partial_trace = clip_path_to_trigger(snapshot.points, candidate)
        merge_trace(result.executed_trace, partial_trace)
        stop_point = partial_trace[-1]
        current_position = (stop_point['x'], stop_point['y'])

        triggered_obstacle = next((obstacle for obstacle in physical_obstacles if obstacle['id'] == candidate.obstacle_id), None)
        if triggered_obstacle is None:
            result.failure_reason = "触发点未找到对应障碍物"
            return result

        if not obstacle_visible_from_position(triggered_obstacle, current_position, radius):
            result.failure_reason = "触发障碍物不在机器狗当前视域内"
            return result

        packets = [build_semantic_packet(triggered_obstacle)]
        visible_ids = [triggered_obstacle['id']]
        result.discovered_ids.add(triggered_obstacle['id'])
        result.events.append(
            TriggerEvent(
                index=len(result.events) + 1,
                stop_point=stop_point,
                triggered_obstacle_id=candidate.obstacle_id,
                uploaded_packets=packets,
                visible_ids=visible_ids,
                source_plan=snapshot.label,
            )
        )

        if point_distance(current_position, goal) < 1.0:
            result.success = True
            return result

    result.failure_reason = "超过最大重规划次数，疑似陷入循环"
    return result


def create_fusion_figure(result: ScenarioResult, output_path: str):
    config = result.config
    map_size = (config['map_size']['x'], config['map_size']['y'])
    start = (config['start']['x'], config['start']['y'])
    goal = (config['goal']['x'], config['goal']['y'])
    radius = config.get('perception_radius', 12)

    fig, axes = plt.subplots(2, 2, figsize=(20, 16))
    physical_obstacles = get_physical_obstacles(config)
    uav_obstacles = build_uav_perspective_obstacles(config)
    dog_semantic_obstacles = build_discovered_semantic_map(config, result.discovered_ids)
    fused_visual_obstacles = build_fused_visual_obstacles(config, result.discovered_ids)

    ax = axes[0, 0]
    for obstacle in physical_obstacles:
        draw_obstacle(ax, obstacle, view='physical', discovered=True)
    if result.executed_trace:
        draw_path(ax, result.executed_trace, COLORS['executed'], 'Executed trace', linewidth=3.0)
    draw_event_overlays(ax, result.events, radius)
    decorate_axes(ax, 'Physical Map', map_size, start, goal)

    ax = axes[0, 1]
    for obstacle in uav_obstacles:
        draw_obstacle(ax, obstacle, view='uav', discovered=False)
    if result.initial_plan:
        draw_path(ax, result.initial_plan.points, COLORS['initial_path'], f'{result.initial_plan.label} Initial Path')
    decorate_axes(ax, 'UAV Perspective Map', map_size, start, goal)

    ax = axes[1, 0]
    for obstacle in dog_semantic_obstacles:
        draw_obstacle(ax, obstacle, view='dog', discovered=True)
    if result.executed_trace:
        draw_path(ax, result.executed_trace, COLORS['executed'], 'Executed trace', linewidth=3.0)
    draw_event_overlays(ax, result.events, radius)
    decorate_axes(ax, 'Dog Semantic Discoveries', map_size, start, goal)

    ax = axes[1, 1]
    for obstacle in fused_visual_obstacles:
        draw_obstacle(ax, obstacle, view='fused', discovered=obstacle['id'] in result.discovered_ids)
    if result.initial_plan:
        draw_path(ax, result.initial_plan.points, COLORS['initial_path'], f'{result.initial_plan.label}', linestyle='--', alpha=0.7)
    if result.final_plan:
        draw_path(ax, result.final_plan.points, COLORS['final_path'], f'{result.final_plan.label} Final Path', linewidth=3.0)
    if result.executed_trace:
        draw_path(ax, result.executed_trace, COLORS['executed'], 'Executed trace', linewidth=2.8)
    draw_event_overlays(ax, result.events, radius)
    decorate_axes(ax, 'UAV Fused Map', map_size, start, goal)

    legend_handles = [
        mpatches.Patch(facecolor=COLORS['building'], edgecolor='#2E2723', label='Standard obstacle'),
        mpatches.Patch(facecolor=COLORS['crawl_fill'], edgecolor=COLORS['crawl_edge'], hatch='///', label='Dog-passable wall'),
        mpatches.Patch(facecolor=COLORS['swamp_fill'], edgecolor=COLORS['swamp_edge'], hatch='xx', label='Swamp / quicksand / water'),
        plt.Line2D([0], [0], color=COLORS['initial_path'], linestyle='--', label='Initial path P_0'),
        plt.Line2D([0], [0], color=COLORS['final_path'], linestyle='-', label='Final replanned path'),
        plt.Line2D([0], [0], color=COLORS['executed'], linestyle='-', label='Executed trace'),
    ]
    fig.legend(handles=legend_handles, loc='lower center', ncol=3, fontsize=10)
    fig.suptitle(f"{config['name']}\nTangent Trigger + Semantic Upload + Fused Replanning", fontsize=16, fontweight='bold')
    plt.tight_layout(rect=(0, 0.04, 1, 0.97))
    plt.savefig(output_path, dpi=160, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def create_metrics_figure(result: ScenarioResult, output_path: str):
    plan_labels = [plan.label for plan in result.plans]
    plan_lengths = [plan.metrics.get('path_length', 0.0) for plan in result.plans]
    waypoints = [plan.metrics.get('num_waypoints', 0) for plan in result.plans]
    executed_length = PathMetrics.path_length(result.executed_trace)
    initial_length = result.initial_plan.metrics.get('path_length', 0.0) if result.initial_plan else 0.0
    final_length = result.final_plan.metrics.get('path_length', 0.0) if result.final_plan else 0.0

    crawl_updates = 0
    swamp_updates = 0
    for event in result.events:
        for packet in event.uploaded_packets:
            if packet['semantic_type'] in CRAWL_TYPES:
                crawl_updates += 1
            if packet['semantic_type'] in SWAMP_TYPES:
                swamp_updates += 1

    fig, axes = plt.subplots(2, 2, figsize=(16, 11))

    axes[0, 0].plot(plan_labels, plan_lengths, marker='o', color=COLORS['final_path'], linewidth=2.5)
    axes[0, 0].set_title('Path Length After Each Replan')
    axes[0, 0].set_ylabel('Length (m)')
    axes[0, 0].grid(True, alpha=0.25)

    axes[0, 1].bar(plan_labels, waypoints, color=COLORS['initial_path'], alpha=0.8)
    axes[0, 1].set_title('Waypoints Per Plan')
    axes[0, 1].set_ylabel('Waypoints')
    axes[0, 1].grid(True, axis='y', alpha=0.25)

    compare_labels = ['Initial plan', 'Final plan', 'Executed trace']
    compare_values = [initial_length, final_length, executed_length]
    axes[1, 0].bar(compare_labels, compare_values,
                   color=[COLORS['initial_path'], COLORS['final_path'], COLORS['executed']], alpha=0.85)
    axes[1, 0].set_title('Length Comparison')
    axes[1, 0].set_ylabel('Length (m)')
    axes[1, 0].grid(True, axis='y', alpha=0.25)

    event_labels = ['Triggers', 'Uploads', 'Low-wall fixes', 'Swamp fixes']
    event_values = [len(result.events), sum(len(event.uploaded_packets) for event in result.events), crawl_updates, swamp_updates]
    axes[1, 1].bar(event_labels, event_values,
                   color=[COLORS['circle'], COLORS['event'], COLORS['crawl_edge'], COLORS['swamp_edge']], alpha=0.85)
    axes[1, 1].set_title('Cooperation Events')
    axes[1, 1].grid(True, axis='y', alpha=0.25)

    plt.suptitle(f"{result.config['name']} - Replanning Metrics", fontsize=15, fontweight='bold')
    plt.tight_layout(rect=(0, 0, 1, 0.96))
    plt.savefig(output_path, dpi=160, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def write_report(result: ScenarioResult, output_path: str):
    config = result.config
    physical_obstacles = get_physical_obstacles(config)
    crawl_ids = [obstacle['id'] for obstacle in physical_obstacles if obstacle_semantic_type(obstacle) in CRAWL_TYPES]
    swamp_ids = [obstacle['id'] for obstacle in physical_obstacles if obstacle_semantic_type(obstacle) in SWAMP_TYPES]

    with open(output_path, 'w', encoding='utf-8') as file:
        file.write(f"Scenario: {config['name']}\n")
        file.write(f"Description: {config['description']}\n")
        file.write(f"Map Size: {config['map_size']['x']} x {config['map_size']['y']}\n")
        file.write(f"Start: ({config['start']['x']}, {config['start']['y']})\n")
        file.write(f"Goal: ({config['goal']['x']}, {config['goal']['y']})\n")
        file.write(f"Perception Radius: {config.get('perception_radius', 12)}\n\n")

        file.write("=== Semantic Obstacles ===\n")
        file.write(f"Dog-passable walls: {crawl_ids}\n")
        file.write(f"Swamp / quicksand / water: {swamp_ids}\n\n")

        file.write("=== Planning Summary ===\n")
        for plan in result.plans:
            file.write(f"{plan.label}: length={plan.metrics.get('path_length')}m, ")
            file.write(f"waypoints={plan.metrics.get('num_waypoints')}, ")
            file.write(f"turns={plan.metrics.get('num_turns')}, ")
            file.write(f"time={plan.metrics.get('computation_time_ms')}ms, ")
            file.write(f"obstacles={plan.obstacle_count}\n")
        file.write("\n")

        file.write("=== Trigger Events ===\n")
        if not result.events:
            file.write("No trigger event. Initial UAV path reached goal directly.\n")
        for event in result.events:
            file.write(f"E{event.index} @ ({event.stop_point['x']:.2f}, {event.stop_point['y']:.2f})\n")
            file.write(f"  source plan: {event.source_plan}\n")
            file.write(f"  triggered by: {event.triggered_obstacle_id}\n")
            file.write(f"  visible ids: {event.visible_ids}\n")
            for packet in event.uploaded_packets:
                semantic_name = format_semantic_name(packet['semantic_type'])
                file.write(f"    - packet {packet['id']}: {semantic_name}\n")
                file.write(f"      traversable={packet['payload'].get('traversable')}\n")
                if packet['payload'].get('properties'):
                    file.write(f"      properties={packet['payload']['properties']}\n")
            file.write("\n")

        file.write("=== Final Result ===\n")
        file.write(f"Success: {result.success}\n")
        if result.failure_reason:
            file.write(f"Failure Reason: {result.failure_reason}\n")
        file.write(f"Discovered semantic obstacle ids: {sorted(result.discovered_ids)}\n")
        file.write(f"Executed trace length: {PathMetrics.path_length(result.executed_trace):.2f}m\n")
        if result.initial_plan:
            file.write(f"Initial path length: {result.initial_plan.metrics.get('path_length')}m\n")
        if result.final_plan:
            file.write(f"Final path length: {result.final_plan.metrics.get('path_length')}m\n")


def scenario_list(base_dir: str) -> List[Tuple[str, str]]:
    return [
        ("scenario_a_simple", os.path.join(base_dir, "config/scenarios/scenario_a_simple.json")),
        ("scenario_b_maze", os.path.join(base_dir, "config/scenarios/scenario_b_maze.json")),
        ("scenario_c_complex", os.path.join(base_dir, "config/scenarios/scenario_c_complex.json")),
        ("scenario_d_decision_complexity", os.path.join(base_dir, "config/scenarios/scenario_d_decision_complexity.json")),
    ]


def main():
    base_dir = os.path.dirname(__file__)
    print("=" * 80)
    print("事件驱动地空协同仿真")
    print("机制：相切即触发、语义上传、融合重规划")
    print("=" * 80)

    for scenario_id, scenario_path in scenario_list(base_dir):
        config = load_scenario(scenario_path)
        print(f"\n{'=' * 70}")
        print(f"Scenario: {config['name']}")
        print(f"Description: {config['description']}")

        result = simulate_scenario(config)
        output_dir = os.path.join(base_dir, "output", scenario_id)
        os.makedirs(output_dir, exist_ok=True)

        create_fusion_figure(result, os.path.join(output_dir, f"{scenario_id}_fusion.png"))
        create_metrics_figure(result, os.path.join(output_dir, f"{scenario_id}_metrics.png"))
        write_report(result, os.path.join(output_dir, f"{scenario_id}_report.txt"))

        print(f"  success: {result.success}")
        print(f"  triggers: {len(result.events)}")
        print(f"  discovered semantic obstacles: {sorted(result.discovered_ids)}")
        if result.final_plan:
            print(f"  final path length: {result.final_plan.metrics.get('path_length')}m")
        if result.failure_reason:
            print(f"  failure reason: {result.failure_reason}")

    print(f"\n{'=' * 80}")
    print("All scenarios completed.")
    print("=" * 80)


if __name__ == "__main__":
    main()
