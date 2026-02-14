#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Metrics Calculator

路径评估指标计算模块。
支持多指标评估体系：
1. 路径长度（主要指标）
2. 转弯次数
3. 平滑度
4. 穿过的区域数
5. 计算时间
6. 最优性比率
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
    def regions_crossed(points: List[Dict],
                        grid_size: float = 10.0) -> int:
        """
        计算路径穿过的区域数（将地图划分为网格）

        Args:
            points: 路径点列表
            grid_size: 网格大小（米）

        Returns:
            穿过的网格数量
        """
        if len(points) < 2:
            return 0

        visited_regions = set()

        for point in points:
            grid_x = int(point['x'] / grid_size)
            grid_y = int(point['y'] / grid_size)
            visited_regions.add((grid_x, grid_y))

        return len(visited_regions)

    @staticmethod
    def straight_line_distance(start: Dict, goal: Dict) -> float:
        """计算起点到终点的直线距离"""
        dx = goal['x'] - start['x']
        dy = goal['y'] - start['y']
        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def efficiency_ratio(calculated_length: float,
                         straight_distance: float) -> float:
        """
        计算路径效率比率

        Args:
            calculated_length: 实际路径长度
            straight_distance: 直线距离

        Returns:
            效率比率 (1.0 = 直线)
        """
        if straight_distance < 0.001:
            return 1.0
        return straight_distance / calculated_length

    @staticmethod
    def calculate_all(path_result: Dict,
                      start_time: float,
                      end_time: float,
                      theoretical_distance: float = None,
                      map_size: Tuple = None) -> Dict:
        """
        计算所有指标

        Args:
            path_result: 路径规划结果
            start_time: 开始时间
            end_time: 结束时间
            theoretical_distance: 理论最短距离
            map_size: 地图尺寸 (width, height)

        Returns:
            包含所有指标的字典
        """
        points = path_result.get('points', [])

        length = PathMetrics.path_length(points)
        waypoints = PathMetrics.num_waypoints(points)
        turns = PathMetrics.num_turns(points)
        time_ms = (end_time - start_time) * 1000
        smoothness = PathMetrics.smoothness(points)

        if map_size:
            grid_size = min(map_size) / 10
            regions = PathMetrics.regions_crossed(points, grid_size)
        else:
            regions = None

        if theoretical_distance:
            optimality = PathMetrics.optimality_ratio(length, theoretical_distance)
            efficiency = PathMetrics.efficiency_ratio(length, theoretical_distance)
        else:
            optimality = None
            efficiency = None

        return {
            'path_length': round(length, 2),
            'num_waypoints': waypoints,
            'num_turns': turns,
            'computation_time_ms': round(time_ms, 3),
            'smoothness': round(smoothness, 3),
            'optimality_ratio': round(optimality, 3) if optimality else None,
            'efficiency_ratio': round(efficiency, 3) if efficiency else None,
            'regions_crossed': regions
        }

    @staticmethod
    def compare_paths(metrics_uav: Dict, metrics_dog: Dict) -> Dict:
        """
        比较两条路径的指标

        Args:
            metrics_uav: UAV路径指标
            metrics_dog: Dog路径指标

        Returns:
            比较结果字典
        """
        result = {
            'length_diff': None,
            'length_diff_pct': None,
            'turn_diff': None,
            'smoothness_diff': None,
            'time_diff': None,
            'winner': None,
            'reason': []
        }

        if not metrics_uav or not metrics_dog:
            return result

        length_uav = metrics_uav.get('path_length', 0)
        length_dog = metrics_dog.get('path_length', 0)

        if length_uav > 0 and length_dog > 0:
            length_diff = length_uav - length_dog
            result['length_diff'] = round(length_diff, 2)
            result['length_diff_pct'] = round(100 * length_diff / length_dog, 2)

            if abs(length_diff) / length_dog > 0.05:
                if length_diff < 0:
                    result['winner'] = 'uav'
                    result['reason'].append(f"UAV路径更短 {-length_diff:.2f}m")
                else:
                    result['winner'] = 'dog'
                    result['reason'].append(f"Dog路径更短 {length_diff:.2f}m")

        turns_uav = metrics_uav.get('num_turns', 0)
        turns_dog = metrics_dog.get('num_turns', 0)
        result['turn_diff'] = turns_uav - turns_dog

        smoothness_uav = metrics_uav.get('smoothness', 0)
        smoothness_dog = metrics_dog.get('smoothness', 0)
        result['smoothness_diff'] = round(smoothness_uav - smoothness_dog, 3)

        time_uav = metrics_uav.get('computation_time_ms', 0)
        time_dog = metrics_dog.get('computation_time_ms', 0)
        result['time_diff'] = round(time_uav - time_dog, 3)

        if not result['reason']:
            if result['turn_diff'] < 0:
                result['winner'] = 'uav'
                result['reason'].append(f"UAV转弯次数更少 {-result['turn_diff']}次")
            elif result['turn_diff'] > 0:
                result['winner'] = 'dog'
                result['reason'].append(f"Dog转弯次数更少 {result['turn_diff']}次")

            if result['smoothness_diff'] > 0.05:
                if not result['winner']:
                    result['winner'] = 'uav'
                result['reason'].append(f"UAV路径更平滑 {result['smoothness_diff']:.3f}")
            elif result['smoothness_diff'] < -0.05:
                if not result['winner']:
                    result['winner'] = 'dog'
                result['reason'].append(f"Dog路径更平滑 {-result['smoothness_diff']:.3f}")

        if not result['winner']:
            result['winner'] = 'tie'
            result['reason'].append("两条路径指标相近")

        result['reason'] = result['reason'] if result['reason'] else ["无法确定更优路径"]

        return result


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
