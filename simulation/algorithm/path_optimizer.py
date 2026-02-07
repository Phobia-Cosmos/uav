#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Optimizer

路径优化模块：RDP 简化 + B样条平滑。
"""

import math
from typing import List, Tuple, Dict, Optional
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))


def rdp_simplify(points: List[Tuple[float, float]],
                 epsilon: float = 1.0) -> List[Tuple[float, float]]:
    """
    Ramer-Douglas-Peucker 算法路径简化

    去除路径中的冗余点，保留关键转折

    Args:
        points: 原始路径点列表 [(x, y), ...]
        epsilon: 简化阈值（距离阈值）

    Returns:
        简化后的路径点列表

    Example:
        >>> points = [(0,0), (1,0), (2,0), (3,0), (3,3)]
        >>> rdp_simplify(points, epsilon=1.0)
        [(0,0), (3,0), (3,3)]
    """
    if len(points) < 3:
        return points

    def perpendicular_distance(point: Tuple[float, float],
                              line_start: Tuple[float, float],
                              line_end: Tuple[float, float]) -> float:
        """计算点到线段的垂直距离"""
        x, y = point
        x1, y1 = line_start
        x2, y2 = line_end

        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            return math.sqrt((x - x1) ** 2 + (y - y1) ** 2)

        t = max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)))

        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        return math.sqrt((x - closest_x) ** 2 + (y - closest_y) ** 2)

    def rdp_recursive(pts: List[Tuple[float, float]],
                      start: int, end: int, kept: List[bool]) -> float:
        """递归 RDP 简化"""
        if end <= start + 1:
            return 0.0

        max_dist = 0.0
        max_idx = start

        for i in range(start + 1, end):
            d = perpendicular_distance(pts[i], pts[start], pts[end])
            if d > max_dist:
                max_dist = d
                max_idx = i

        if max_dist > epsilon:
            rdp_recursive(pts, start, max_idx, kept)
            rdp_recursive(pts, max_idx, end, kept)
        else:
            kept[start] = True
            kept[end] = True

    kept = [False] * len(points)
    kept[0] = True
    kept[-1] = True

    rdp_recursive(points, 0, len(points) - 1, kept)

    return [points[i] for i in range(len(points)) if kept[i]]


def bspline_smooth(points: List[Tuple[float, float]],
                   num_samples: int = 100) -> List[Tuple[float, float]]:
    """
    三次 B 样条曲线平滑

    生成通过控制点的平滑曲线

    Args:
        points: 控制点列表（简化后的关键点）
        num_samples: 输出曲线的采样点数

    Returns:
        平滑后的曲线点列表
    """
    if len(points) < 2:
        return points

    def basis_function(t: float,
                       p0: Tuple[float, float],
                       p1: Tuple[float, float],
                       p2: Tuple[float, float],
                       p3: Tuple[float, float]) -> Tuple[float, float]:
        """三次 B 样条基函数"""
        t2 = t * t
        t3 = t2 * t

        b0 = (-t3 + 3 * t2 - 3 * t + 1) / 6
        b1 = (3 * t3 - 6 * t2 + 4) / 6
        b2 = (-3 * t3 + 3 * t2 + 3 * t + 1) / 6
        b3 = t3 / 6

        return (
            b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0],
            b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1]
        )

    if len(points) == 2:
        return points

    if len(points) == 3:
        p0, p1, p2 = points
        return [
            basis_function(t, p0, p1, p1, p2)
            for t in [i / (num_samples - 1) for i in range(num_samples)]
        ]

    smoothed = []

    for i in range(len(points) - 3):
        p0 = points[i]
        p1 = points[i + 1]
        p2 = points[i + 2]
        p3 = points[i + 3]

        if i == 0:
            samples = num_samples // (len(points) - 2)
        else:
            samples = num_samples // (len(points) - 2)

        for j in range(samples):
            t = j / samples if samples > 1 else 0.5
            smoothed.append(basis_function(t, p0, p1, p2, p3))

    smoothed.append(points[-1])

    return smoothed


def calculate_turn_angles(points: List[Tuple[float, float]]) -> List[float]:
    """
    计算路径中每个点的转向角度

    Returns:
        转向角度列表（弧度）
    """
    angles = []

    for i in range(1, len(points) - 1):
        x1, y1 = points[i - 1]
        x2, y2 = points[i]
        x3, y3 = points[i + 1]

        v1 = (x2 - x1, y2 - y1)
        v2 = (x3 - x2, y3 - y2)

        dot = v1[0] * v2[0] + v1[1] * v2[1]
        mag1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
        mag2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

        if mag1 < 0.001 or mag2 < 0.001:
            angles.append(0)
            continue

        cos_angle = max(-1, min(1, dot / (mag1 * mag2)))
        angle = math.acos(cos_angle)

        angles.append(angle)

    return angles


def calculate_smoothness(points: List[Tuple[float, float]]) -> float:
    """
    计算路径平滑度评分 (0-1)

    评分标准：
    - 1.0: 完全直线
    - 0.8-1.0: 平滑曲线
    - 0.6-0.8: 基本平滑
    - 0.4-0.6: 一般
    - 0.0-0.4: 锯齿严重

    Returns:
        平滑度评分 (0-1)
    """
    if len(points) < 3:
        return 1.0

    angles = calculate_turn_angles(points)

    if len(angles) == 0:
        return 1.0

    max_angle = max(angles) if angles else 0
    avg_angle = sum(angles) / len(angles) if angles else 0

    angle_penalty = max_angle / math.pi

    angle_consistency = 1.0 - (max_angle - avg_angle) / (math.pi / 2) if max_angle > avg_angle else 1.0

    smoothness = (1 - angle_penalty) * angle_consistency

    return max(0, min(1, smoothness))


def calculate_path_length(points: List[Tuple[float, float]]) -> float:
    """计算路径总长度"""
    if len(points) < 2:
        return 0.0

    total = 0.0
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        total += math.sqrt(dx * dx + dy * dy)

    return total


def count_turns(points: List[Tuple[float, float]],
                threshold: float = math.pi / 6) -> int:
    """
    计算路径中的转折次数

    Args:
        points: 路径点列表
        threshold: 角度阈值（小于此值不视为转折）

    Returns:
        转折次数
    """
    if len(points) < 3:
        return 0

    angles = calculate_turn_angles(points)
    turns = sum(1 for angle in angles if angle > threshold)

    return turns


class PathOptimizer:
    """路径优化器"""

    def __init__(self, rdp_epsilon: float = 1.5,
                 bspline_samples: int = 50,
                 smoothness_threshold: float = 0.7):
        """
        初始化路径优化器

        Args:
            rdp_epsilon: RDP 简化阈值
            bspline_samples: B样条曲线采样点数
            smoothness_threshold: 目标平滑度阈值
        """
        self.rdp_epsilon = rdp_epsilon
        self.bspline_samples = bspline_samples
        self.smoothness_threshold = smoothness_threshold

    def optimize(self, points: List[Tuple[float, float]]) -> Dict:
        """
        执行完整的路径优化

        Args:
            points: 原始路径点列表

        Returns:
            优化结果字典
        """
        original_length = calculate_path_length(points)
        original_smoothness = calculate_smoothness(points)
        original_turns = count_turns(points)

        simplified = rdp_simplify(points, self.rdp_epsilon)
        simplified_length = calculate_path_length(simplified)
        simplified_turns = count_turns(simplified)

        if len(simplified) >= 2:
            smoothed = bspline_smooth(simplified, self.bspline_samples)
            smoothed_length = calculate_path_length(smoothed)
            smoothed_smoothness = calculate_smoothness(smoothed)
            smoothed_turns = count_turns(smoothed)
        else:
            smoothed = simplified
            smoothed_length = simplified_length
            smoothed_smoothness = simplified_turns
            smoothed_turns = 0

        return {
            "original": {
                "points": points,
                "length": original_length,
                "num_points": len(points),
                "smoothness": original_smoothness,
                "turns": original_turns
            },
            "simplified": {
                "points": simplified,
                "length": simplified_length,
                "num_points": len(simplified),
                "smoothness": calculate_smoothness(simplified),
                "turns": simplified_turns
            },
            "smoothed": {
                "points": smoothed,
                "length": smoothed_length,
                "num_points": len(smoothed),
                "smoothness": smoothed_smoothness,
                "turns": smoothed_turns
            }
        }


def main():
    """测试路径优化"""
    test_points = [
        (0, 0), (1, 0), (2, 0), (3, 0), (4, 0),
        (4, 1), (4, 2), (4, 3), (4, 4), (5, 4),
        (6, 4), (7, 4), (8, 4), (8, 5), (8, 6),
        (8, 7), (8, 8), (9, 8), (10, 8)
    ]

    print("Testing Path Optimization...")
    print(f"Original points: {len(test_points)}")

    simplified = rdp_simplify(test_points, epsilon=1.5)
    print(f"Simplified points: {len(simplified)}")

    smoothed = bspline_smooth(simplified, num_samples=50)
    print(f"Smoothed points: {len(smoothed)}")

    print(f"\nOriginal smoothness: {calculate_smoothness(test_points):.3f}")
    print(f"Simplified smoothness: {calculate_smoothness(simplified):.3f}")
    print(f"Smoothed smoothness: {calculate_smoothness(smoothed):.3f}")

    optimizer = PathOptimizer()
    result = optimizer.optimize(test_points)
    print(f"\nOptimization result:")
    print(f"  Original: {result['original']['num_points']} points, {result['original']['length']:.2f}m")
    print(f"  Simplified: {result['simplified']['num_points']} points, {result['simplified']['length']:.2f}m")
    print(f"  Smoothed: {result['smoothed']['num_points']} points, {result['smoothed']['length']:.2f}m")


if __name__ == "__main__":
    main()
