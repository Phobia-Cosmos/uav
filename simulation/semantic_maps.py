#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Semantic map helpers for ground-air cooperation simulation.

负责三张地图的生成、语义修正与“相切触发”事件检测。
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

from map_generator import Obstacle2D


SWAMP_TYPES = {"swamp", "quicksand", "water"}
CRAWL_TYPES = {"crawl_under_wall", "low_wall"}
SPECIAL_TYPES = SWAMP_TYPES | CRAWL_TYPES


@dataclass
class TriggerCandidate:
    obstacle_id: str
    point: Dict[str, float]
    distance_along_path: float
    segment_index: int


def get_physical_obstacles(config: Dict) -> List[Dict]:
    return config.get("physical_obstacles", config.get("obstacles", []))


def obstacle_object(obstacle: Dict) -> Obstacle2D:
    return Obstacle2D.from_dict(obstacle)


def obstacle_semantic_type(obstacle: Dict) -> str:
    return obstacle.get("semantic_type", "generic")


def is_special_obstacle(obstacle: Dict) -> bool:
    return obstacle_semantic_type(obstacle) in SPECIAL_TYPES


def obstacle_blocks_for_uav(obstacle: Dict) -> bool:
    semantic_type = obstacle_semantic_type(obstacle)
    if semantic_type in SWAMP_TYPES:
        return False
    return True


def obstacle_blocks_for_dog_truth(obstacle: Dict) -> bool:
    semantic_type = obstacle_semantic_type(obstacle)
    if semantic_type in CRAWL_TYPES:
        return False
    if semantic_type in SWAMP_TYPES:
        return True
    return True


def obstacle_visible_from_position(obstacle: Dict,
                                   position: Tuple[float, float],
                                   radius: float) -> bool:
    return obstacle_object(obstacle).distance_to_point(position) <= radius + 1e-6


def build_uav_perspective_obstacles(config: Dict) -> List[Dict]:
    obstacles = []
    for obstacle in get_physical_obstacles(config):
        if obstacle_blocks_for_uav(obstacle):
            obstacles.append(copy.deepcopy(obstacle))
    return obstacles


def build_fused_planning_obstacles(config: Dict, discovered_ids: Set[str]) -> List[Dict]:
    obstacles = []
    for obstacle in get_physical_obstacles(config):
        obstacle_id = obstacle["id"]
        if obstacle_id in discovered_ids:
            if obstacle_blocks_for_dog_truth(obstacle):
                obstacles.append(copy.deepcopy(obstacle))
        elif obstacle_blocks_for_uav(obstacle):
            obstacles.append(copy.deepcopy(obstacle))
    return obstacles


def build_dog_visible_truth(config: Dict,
                            position: Tuple[float, float],
                            radius: float) -> List[Dict]:
    visible = []
    for obstacle in get_physical_obstacles(config):
        if obstacle_visible_from_position(obstacle, position, radius):
            visible.append(copy.deepcopy(obstacle))
    return visible


def build_discovered_semantic_map(config: Dict,
                                  discovered_ids: Set[str]) -> List[Dict]:
    visible = []
    for obstacle in get_physical_obstacles(config):
        if obstacle["id"] in discovered_ids:
            visible.append(copy.deepcopy(obstacle))
    return visible


def build_fused_visual_obstacles(config: Dict,
                                 discovered_ids: Set[str]) -> List[Dict]:
    visible = []
    for obstacle in get_physical_obstacles(config):
        obstacle_id = obstacle["id"]
        if obstacle_id in discovered_ids:
            visible.append(copy.deepcopy(obstacle))
        elif obstacle_blocks_for_uav(obstacle):
            visible.append(copy.deepcopy(obstacle))
    return visible


def get_newly_visible_special_obstacles(config: Dict,
                                        position: Tuple[float, float],
                                        radius: float,
                                        discovered_ids: Set[str]) -> List[Dict]:
    visible = []
    for obstacle in get_physical_obstacles(config):
        obstacle_id = obstacle["id"]
        if obstacle_id in discovered_ids or not is_special_obstacle(obstacle):
            continue
        if obstacle_visible_from_position(obstacle, position, radius):
            visible.append(copy.deepcopy(obstacle))
    return visible


def build_semantic_packet(obstacle: Dict) -> Dict:
    semantic_type = obstacle_semantic_type(obstacle)
    packet = {
        "id": obstacle["id"],
        "semantic_type": semantic_type,
        "geometry": obstacle.get("type"),
        "payload": {
            "properties": obstacle.get("properties", {}),
        },
    }

    if obstacle.get("type") == "wall":
        packet["payload"]["start_point"] = obstacle["start_point"]
        packet["payload"]["end_point"] = obstacle["end_point"]
        packet["payload"]["width"] = obstacle.get("width", 1)
    elif obstacle.get("type") == "polygon":
        packet["payload"]["vertices"] = obstacle.get("vertices", [])
    else:
        packet["payload"]["position"] = obstacle.get("position")
        packet["payload"]["size"] = obstacle.get("size", {})

    if semantic_type in CRAWL_TYPES:
        packet["payload"]["traversable"] = True
        packet["payload"]["render_hint"] = "dashed-underpass"
    elif semantic_type in SWAMP_TYPES:
        packet["payload"]["traversable"] = False
        packet["payload"]["render_hint"] = "swamp-hatched"
    else:
        packet["payload"]["traversable"] = False
        packet["payload"]["render_hint"] = "solid"

    return packet


def interpolate_point(start: Sequence[float], end: Sequence[float], ratio: float) -> Tuple[float, float]:
    return (
        start[0] + (end[0] - start[0]) * ratio,
        start[1] + (end[1] - start[1]) * ratio,
    )


def point_distance(point_a: Sequence[float], point_b: Sequence[float]) -> float:
    return math.sqrt((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2)


def _refine_tangent_point(obstacle: Dict,
                          point_a: Tuple[float, float],
                          point_b: Tuple[float, float],
                          radius: float) -> Tuple[float, float]:
    obstacle_model = obstacle_object(obstacle)
    left = point_a
    right = point_b

    for _ in range(24):
        mid = interpolate_point(left, right, 0.5)
        if obstacle_model.distance_to_point(mid) <= radius:
            right = mid
        else:
            left = mid
    return right


def find_next_trigger(path_points: List[Dict],
                      config: Dict,
                      radius: float,
                      discovered_ids: Set[str],
                      step: float = 0.25) -> Optional[TriggerCandidate]:
    if len(path_points) < 2:
        return None

    special_obstacles = [
        obstacle for obstacle in get_physical_obstacles(config)
        if obstacle["id"] not in discovered_ids and is_special_obstacle(obstacle)
    ]
    if not special_obstacles:
        return None

    cumulative = 0.0
    for segment_index in range(1, len(path_points)):
        start_point = (path_points[segment_index - 1]["x"], path_points[segment_index - 1]["y"])
        end_point = (path_points[segment_index]["x"], path_points[segment_index]["y"])
        segment_length = point_distance(start_point, end_point)
        if segment_length < 1e-9:
            continue

        samples = max(1, int(math.ceil(segment_length / step)))
        previous_point = start_point
        previous_visibility = {
            obstacle["id"]: obstacle_visible_from_position(obstacle, previous_point, radius)
            for obstacle in special_obstacles
        }

        for sample_index in range(1, samples + 1):
            ratio = sample_index / samples
            current_point = interpolate_point(start_point, end_point, ratio)

            earliest_candidate: Optional[TriggerCandidate] = None
            for obstacle in special_obstacles:
                obstacle_id = obstacle["id"]
                current_visibility = obstacle_visible_from_position(obstacle, current_point, radius)

                if not previous_visibility[obstacle_id] and current_visibility:
                    tangent_point = _refine_tangent_point(obstacle, previous_point, current_point, radius)
                    local_distance = cumulative + point_distance(start_point, tangent_point)
                    candidate = TriggerCandidate(
                        obstacle_id=obstacle_id,
                        point={"x": tangent_point[0], "y": tangent_point[1]},
                        distance_along_path=local_distance,
                        segment_index=segment_index,
                    )
                    if earliest_candidate is None or candidate.distance_along_path < earliest_candidate.distance_along_path:
                        earliest_candidate = candidate

                previous_visibility[obstacle_id] = current_visibility

            if earliest_candidate is not None:
                return earliest_candidate

            previous_point = current_point

        cumulative += segment_length

    return None


def clip_path_to_trigger(path_points: List[Dict], candidate: TriggerCandidate) -> List[Dict]:
    clipped = [copy.deepcopy(path_points[0])]
    for index in range(1, candidate.segment_index):
        clipped.append(copy.deepcopy(path_points[index]))

    trigger_point = {"x": candidate.point["x"], "y": candidate.point["y"]}
    if point_distance((clipped[-1]["x"], clipped[-1]["y"]), (trigger_point["x"], trigger_point["y"])) > 1e-6:
        clipped.append(trigger_point)
    return clipped


def merge_trace(trace: List[Dict], segment: Iterable[Dict]) -> List[Dict]:
    for point in segment:
        if not trace:
            trace.append(copy.deepcopy(point))
            continue
        last = trace[-1]
        if point_distance((last["x"], last["y"]), (point["x"], point["y"])) > 1e-6:
            trace.append(copy.deepcopy(point))
    return trace


def segment_is_obstacle_free(start_point: Sequence[float],
                             end_point: Sequence[float],
                             obstacles: Sequence[Dict],
                             sample_step: float = 0.2,
                             clearance: float = 1e-6) -> bool:
    segment_length = point_distance(start_point, end_point)
    if segment_length < 1e-9:
        return True

    obstacle_models = [
        obstacle if isinstance(obstacle, Obstacle2D) else obstacle_object(obstacle)
        for obstacle in obstacles
    ]
    samples = max(2, int(math.ceil(segment_length / sample_step)))

    for sample_index in range(1, samples):
        ratio = sample_index / samples
        point = interpolate_point(start_point, end_point, ratio)
        if any(obstacle.distance_to_point(point) <= clearance for obstacle in obstacle_models):
            return False

    return True


def simplify_path_with_line_of_sight(path_points: List[Dict],
                                     obstacles: Sequence[Dict],
                                     sample_step: float = 0.2) -> List[Dict]:
    if len(path_points) <= 2:
        return [copy.deepcopy(point) for point in path_points]

    simplified = [copy.deepcopy(path_points[0])]
    anchor_index = 0

    while anchor_index < len(path_points) - 1:
        next_index = len(path_points) - 1
        anchor_point = (path_points[anchor_index]["x"], path_points[anchor_index]["y"])

        while next_index > anchor_index + 1:
            candidate_point = (path_points[next_index]["x"], path_points[next_index]["y"])
            if segment_is_obstacle_free(anchor_point, candidate_point, obstacles, sample_step=sample_step):
                break
            next_index -= 1

        simplified.append(copy.deepcopy(path_points[next_index]))
        anchor_index = next_index

    return simplified
