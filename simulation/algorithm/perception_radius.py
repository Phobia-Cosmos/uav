#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
感知半径截取模块

从完整地图中截取Dog视角的局部地图
"""

import math
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class Point:
    x: float
    y: float

    def distance_to(self, other: 'Point') -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def is_in_circle(self, center: 'Point', radius: float) -> bool:
        return self.distance_to(center) <= radius


@dataclass
class TruncatedObstacle:
    id: str
    type: str
    is_partial: bool
    original_data: Dict
    clipped_points: Optional[List[Point]] = None


def calculate_perception_boundary(start: Point, goal: Point, radius: float) -> Tuple[Optional[Point], bool]:
    """
    计算感知边界点（射线 start→goal 与圆周的交点）

    Returns:
        boundary_point: 边界交点坐标（若目标在半径内则为None）
        goal_in_range: 目标是否在感知范围内
    """
    dist_to_goal = start.distance_to(goal)

    if dist_to_goal <= radius:
        return None, True

    direction_x = (goal.x - start.x) / dist_to_goal
    direction_y = (goal.y - start.y) / dist_to_goal

    boundary_x = start.x + direction_x * radius
    boundary_y = start.y + direction_y * radius

    return Point(boundary_x, boundary_y), False


def is_obstacle_in_circle(obstacle: Dict, center: Point, radius: float) -> bool:
    """
    判断障碍物是否与感知圆相交或在其中
    """
    if obstacle['type'] == 'rectangle':
        pos = obstacle['position']
        p = Point(pos['x'], pos['y'])
        size = obstacle['size']
        half_w = size['width'] / 2
        half_h = size['height'] / 2
        corners = [
            Point(p.x - half_w, p.y - half_h),
            Point(p.x + half_w, p.y - half_h),
            Point(p.x - half_w, p.y + half_h),
            Point(p.x + half_w, p.y + half_h)
        ]
        for corner in corners:
            if corner.distance_to(center) <= radius + math.sqrt(half_w**2 + half_h**2):
                return True
        return False

    elif obstacle['type'] == 'circle':
        pos = obstacle['position']
        p = Point(pos['x'], pos['y'])
        r = obstacle['size']['radius']
        return p.distance_to(center) <= radius + r

    elif obstacle['type'] == 'wall':
        p1 = Point(obstacle['start_point']['x'], obstacle['start_point']['y'])
        p2 = Point(obstacle['end_point']['x'], obstacle['end_point']['y'])
        width = obstacle.get('width', 1)
        for pt in [p1, p2, Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2)]:
            if pt.distance_to(center) <= radius + width:
                return True
        return False

    return False


def truncate_obstacle_to_circle(obstacle: Dict, center: Point, radius: float) -> Optional[Dict]:
    """
    将障碍物截断到感知圆内
    """
    if not is_obstacle_in_circle(obstacle, center, radius):
        return None

    obs_type = obstacle['type']

    if obs_type == 'rectangle':
        pos = obstacle['position']
        size = obstacle['size']
        half_w = size['width'] / 2
        half_h = size['height'] / 2

        new_x = max(pos['x'] - half_w, center.x - radius)
        new_y = max(pos['y'] - half_h, center.y - radius)
        new_w = min(pos['x'] + half_w, center.x + radius) - new_x
        new_h = min(pos['y'] + half_h, center.y + radius) - new_y

        if new_w <= 0 or new_h <= 0:
            return None

        return {
            'id': obstacle['id'],
            'type': 'rectangle',
            'position': {'x': new_x + new_w / 2, 'y': new_y + new_h / 2},
            'size': {'width': new_w, 'height': new_h}
        }

    elif obs_type == 'circle':
        pos = obstacle['position']
        p = Point(pos['x'], pos['y'])
        r = obstacle['size']['radius']

        dist = p.distance_to(center)
        if dist >= radius + r:
            return None

        new_r = min(r, radius - dist)
        if new_r <= 0:
            return None

        return {
            'id': obstacle['id'],
            'type': 'circle',
            'position': pos,
            'size': {'radius': new_r}
        }

    elif obs_type == 'wall':
        p1 = Point(obstacle['start_point']['x'], obstacle['start_point']['y'])
        p2 = Point(obstacle['end_point']['x'], obstacle['end_point']['y'])
        width = obstacle.get('width', 1)

        new_p1_x = max(p1.x, center.x - radius - width)
        new_p1_y = max(p1.y, center.y - radius - width)
        new_p2_x = min(p2.x, center.x + radius + width)
        new_p2_y = min(p2.y, center.y + radius + width)

        if new_p1_x >= new_p2_x or new_p1_y >= new_p2_y:
            return None

        return {
            'id': obstacle['id'],
            'type': 'wall',
            'start_point': {'x': new_p1_x, 'y': new_p1_y},
            'end_point': {'x': new_p2_x, 'y': new_p2_y},
            'width': width
        }

    return None


def generate_dog_map(full_config: Dict) -> Dict:
    """
    从完整地图配置生成Dog视角的局部地图

    Args:
        full_config: 完整地图配置

    Returns:
        Dog视角地图配置
    """
    start = Point(full_config['start']['x'], full_config['start']['y'])
    goal = Point(full_config['goal']['x'], full_config['goal']['y'])
    radius = full_config.get('perception_radius', 30)

    boundary_point, goal_in_range = calculate_perception_boundary(start, goal, radius)

    truncated_obstacles = []
    for obs in full_config.get('obstacles', []):
        truncated = truncate_obstacle_to_circle(obs, start, radius)
        if truncated is not None:
            truncated_obstacles.append(truncated)

    dog_config = {
        'name': full_config['name'] + " (Dog View)",
        'description': full_config.get('description', '') + " - Dog perception radius view",
        'map_size': full_config['map_size'],
        'start': full_config['start'],
        'goal': full_config['goal'],
        'perception_radius': radius,
        'boundary_point': {
            'x': boundary_point.x,
            'y': boundary_point.y
        } if boundary_point else None,
        'goal_in_range': goal_in_range,
        'obstacles': truncated_obstacles,
        'original_obstacle_count': len(full_config.get('obstacles', [])),
        'truncated_obstacle_count': len(truncated_obstacles)
    }

    return dog_config


def get_boundary_point_for_planning(full_config: Dict) -> Optional[Point]:
    """
    获取Dog路径规划时的边界点
    """
    start = Point(full_config['start']['x'], full_config['start']['y'])
    goal = Point(full_config['goal']['x'], full_config['goal']['y'])
    radius = full_config.get('perception_radius', 30)

    boundary_point, _ = calculate_perception_boundary(start, goal, radius)
    return boundary_point
