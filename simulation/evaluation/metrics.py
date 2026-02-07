#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Metrics Calculator

路径评估指标计算模块。
"""

import math
from typing import List, Dict, Tuple, Optional
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))


class PathMetrics:
    """路径评估指标计算"""

    @staticmethod
    def path_length(points: List[Dict]) -> float:
        """计算路径总长度（米）"""
        if len(points) < 2:
            return 0.0

        total = 0.0
        for i in range(1, len(points)):
            dx = points[i]['x'] - points[i-1]['x']
            dy = points[i]['y'] - points[i-1]['y']
            total += math.sqrt(dx * dx + dy * dy)

        return total

    @staticmethod
    def num_waypoints(points: List[Dict]) -> int:
        """计算路径点数"""
        return len(points)

    @staticmethod
    def num_turns(points: List[Dict],
                   threshold: float = math.pi / 12) -> int:
        """
        计算路径转折次数

        Args:
            points: 路径点列表
            threshold: 角度阈值（弧度），小于此值不视为转折

        Returns:
            转折次数
        """
        if len(points) < 3:
            return 0

        turns = 0

        for i in range(1, len(points) - 1):
            x1, y1 = points[i-1]['x'], points[i-1]['y']
            x2, y2 = points[i]['x'], points[i]['y']
            x3, y3 = points[i+1]['x'], points[i+1]['y']

            v1 = (x2 - x1, y2 - y1)
            v2 = (x3 - x2, y3 - y2)

            dot = v1[0] * v2[0] + v1[1] * v2[1]
            mag1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
            mag2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

            if mag1 < 0.001 or mag2 < 0.001:
                continue

            cos_angle = max(-1, min(1, dot / (mag1 * mag2)))
            angle = math.acos(cos_angle)

            if angle > threshold:
                turns += 1

        return turns

    @staticmethod
    def computation_time(start_time: float, end_time: float) -> float:
        """计算规划时间（秒）"""
        return end_time - start_time

    @staticmethod
    def smoothness(points: List[Dict]) -> float:
        """
        计算路径平滑度评分 (0-1)

        评分标准：
        - 1.0: 完全直线
        - 0.8-1.0: 平滑曲线
        - 0.6-0.8: 基本平滑
        - 0.4-0.6: 一般
        - 0.0-0.4: 锯齿严重
        """
        if len(points) < 3:
            return 1.0

        angles = []
        for i in range(1, len(points) - 1):
            x1, y1 = points[i-1]['x'], points[i-1]['y']
            x2, y2 = points[i]['x'], points[i]['y']
            x3, y3 = points[i+1]['x'], points[i+1]['y']

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

        if len(angles) == 0:
            return 1.0

        max_angle = max(angles)
        avg_angle = sum(angles) / len(angles)

        angle_penalty = max_angle / math.pi

        angle_consistency = 1.0 - (max_angle - avg_angle) / (math.pi / 2) if max_angle > avg_angle else 1.0

        smoothness = (1 - angle_penalty) * angle_consistency

        return max(0, min(1, smoothness))

    @staticmethod
    def optimality_ratio(calculated: float,
                        theoretical: float) -> float:
        """
        计算最优性比率

        Args:
            calculated: 计算得到的路径长度
            theoretical: 理论最短距离（直线距离）

        Returns:
            最优性比率 (1.0 = 最优)
        """
        if theoretical < 0.001:
            return 1.0 if calculated < 0.001 else 0.0

        return theoretical / calculated if calculated > theoretical else 1.0

    @staticmethod
    def success_rate(total_runs: int, successful_runs: int) -> float:
        """计算成功率"""
        if total_runs == 0:
            return 0.0
        return successful_runs / total_runs

    @staticmethod
    def calculate_all(path_result: Dict,
                     start_time: float,
                     end_time: float,
                     theoretical_distance: float = None) -> Dict:
        """
        计算所有指标

        Args:
            path_result: 路径规划结果
            start_time: 开始时间
            end_time: 结束时间
            theoretical_distance: 理论最短距离

        Returns:
            包含所有指标的字典
        """
        points = path_result.get('points', [])

        length = PathMetrics.path_length(points)
        waypoints = PathMetrics.num_waypoints(points)
        turns = PathMetrics.num_turns(points)
        time_ms = (end_time - start_time) * 1000
        smoothness = PathMetrics.smoothness(points)

        if theoretical_distance:
            optimality = PathMetrics.optimality_ratio(length, theoretical_distance)
        else:
            optimality = None

        return {
            'path_length': round(length, 2),
            'num_waypoints': waypoints,
            'num_turns': turns,
            'computation_time_ms': round(time_ms, 3),
            'smoothness': round(smoothness, 3),
            'optimality_ratio': round(optimality, 3) if optimality else None
        }


def main():
    """测试指标计算"""
    test_path = [
        {'x': 0, 'y': 0},
        {'x': 5, 'y': 0},
        {'x': 10, 'y': 0},
        {'x': 10, 'y': 5},
        {'x': 10, 'y': 10},
        {'x': 15, 'y': 10},
        {'x': 20, 'y': 10}
    ]

    print("Testing Path Metrics...")
    print(f"  Path length: {PathMetrics.path_length(test_path):.2f}m")
    print(f"  Waypoints: {PathMetrics.num_waypoints(test_path)}")
    print(f"  Turns: {PathMetrics.num_turns(test_path)}")
    print(f"  Smoothness: {PathMetrics.smoothness(test_path):.3f}")

    import time
    start = time.time()
    result = {'points': test_path}
    end = time.time()

    metrics = PathMetrics.calculate_all(result, start, end, 28.28)
    print(f"\nAll metrics: {metrics}")


if __name__ == "__main__":
    main()
