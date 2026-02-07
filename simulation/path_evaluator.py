#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Evaluator

路径评分模块。
基于长度、平滑度和能耗计算路径评分。
"""

from typing import Dict, List, Tuple, Any
from dataclasses import dataclass
import math


@dataclass
class EvaluationResult:
    """路径评估结果"""
    total_score: float
    length_score: float
    smoothness_score: float
    energy_score: float
    path_length: float
    turn_count: int
    estimated_time: float

    def to_dict(self) -> Dict:
        return {
            "total_score": self.total_score,
            "length_score": self.length_score,
            "smoothness_score": self.smoothness_score,
            "energy_score": self.energy_score,
            "path_length": self.path_length,
            "turn_count": self.turn_count,
            "estimated_time": self.estimated_time
        }


class PathEvaluator:
    """路径评分器"""

    # 评分权重
    WEIGHT_LENGTH = 0.50
    WEIGHT_SMOOTHNESS = 0.30
    WEIGHT_ENERGY = 0.20

    def __init__(self, grid_size: Tuple[int, int, int] = (50, 50, 30)):
        """
        初始化路径评分器

        Args:
            grid_size: 网格尺寸 (x, y, z)
        """
        self.grid_size = grid_size
        self.max_possible_length = max(grid_size) * 2

    def evaluate_2d(self, path_data: Dict) -> Dict:
        """
        评估 2D 路径 (机器狗)

        Args:
            path_data: 2D 路径数据

        Returns:
            评分结果字典
        """
        points = path_data.get("points", [])
        if len(points) < 2:
            return {"error": "Path too short"}

        # 计算路径长度
        total_length = self._calculate_length_2d(points)

        # 计算转向次数
        turn_count = self._count_turns_2d(points)

        # 估算能耗
        energy = self._estimate_energy_2d(points, turn_count)

        # 计算各项得分
        length_score = self._length_score(total_length)
        smoothness_score = self._smoothness_score(turn_count, len(points))
        energy_score = self._energy_score(energy)

        # 计算总分
        total_score = (
            self.WEIGHT_LENGTH * length_score +
            self.WEIGHT_SMOOTHNESS * smoothness_score +
            self.WEIGHT_ENERGY * energy_score
        )

        result = EvaluationResult(
            total_score=round(total_score, 3),
            length_score=round(length_score, 3),
            smoothness_score=round(smoothness_score, 3),
            energy_score=round(energy_score, 3),
            path_length=round(total_length, 2),
            turn_count=turn_count,
            estimated_time=round(total_length, 2)  # 假设速度 1m/s
        )

        return result.to_dict()

    def evaluate_3d(self, path_data: Dict) -> Dict:
        """
        评估 3D 路径 (无人机)

        Args:
            path_data: 3D 路径数据

        Returns:
            评分结果字典
        """
        points = path_data.get("points", [])
        if len(points) < 2:
            return {"error": "Path too short"}

        # 计算路径长度
        total_length = self._calculate_length_3d(points)

        # 计算转向次数
        turn_count = self._count_turns_3d(points)

        # 估算能耗 (3D 需要更多能量)
        energy = self._estimate_energy_3d(points, turn_count)

        # 计算各项得分
        length_score = self._length_score(total_length)
        smoothness_score = self._smoothness_score(turn_count, len(points))
        energy_score = self._energy_score(energy)

        # 计算总分
        total_score = (
            self.WEIGHT_LENGTH * length_score +
            self.WEIGHT_SMOOTHNESS * smoothness_score +
            self.WEIGHT_ENERGY * energy_score
        )

        result = EvaluationResult(
            total_score=round(total_score, 3),
            length_score=round(length_score, 3),
            smoothness_score=round(smoothness_score, 3),
            energy_score=round(energy_score, 3),
            path_length=round(total_length, 2),
            turn_count=turn_count,
            estimated_time=round(total_length, 2)  # 假设速度 1m/s
        )

        return result.to_dict()

    def _calculate_length_2d(self, points: List[Dict]) -> float:
        """计算 2D 路径总长度"""
        total = 0.0
        for i in range(1, len(points)):
            p1, p2 = points[i-1], points[i]
            dx = p2["x"] - p1["x"]
            dy = p2["y"] - p1["y"]
            total += math.sqrt(dx**2 + dy**2)
        return total

    def _calculate_length_3d(self, points: List[Dict]) -> float:
        """计算 3D 路径总长度"""
        total = 0.0
        for i in range(1, len(points)):
            p1, p2 = points[i-1], points[i]
            dx = p2["x"] - p1["x"]
            dy = p2["y"] - p1["y"]
            dz = p2["z"] - p1["z"]
            total += math.sqrt(dx**2 + dy**2 + dz**2)
        return total

    def _count_turns_2d(self, points: List[Dict]) -> int:
        """计算 2D 路径转向次数"""
        if len(points) < 3:
            return 0

        turns = 0
        for i in range(2, len(points)):
            p0, p1, p2 = points[i-2], points[i-1], points[i]

            # 计算方向向量
            v1 = (p1["x"] - p0["x"], p1["y"] - p0["y"])
            v2 = (p2["x"] - p1["x"], p2["y"] - p1["y"])

            # 检查方向变化
            if v1 != v2:
                turns += 1

        return turns

    def _count_turns_3d(self, points: List[Dict]) -> int:
        """计算 3D 路径转向次数"""
        if len(points) < 3:
            return 0

        turns = 0
        for i in range(2, len(points)):
            p0, p1, p2 = points[i-2], points[i-1], points[i]

            # 计算方向向量
            v1 = (p1["x"] - p0["x"], p1["y"] - p0["y"], p1["z"] - p0["z"])
            v2 = (p2["x"] - p1["x"], p2["y"] - p1["y"], p2["z"] - p1["z"])

            # 检查方向变化
            if v1 != v2:
                turns += 1

        return turns

    def _estimate_energy_2d(self, points: List[Dict], turn_count: int) -> float:
        """
        估算 2D 路径能耗
        基础能耗 + 转向惩罚
        """
        length = self._calculate_length_2d(points)
        turn_penalty = turn_count * 0.5  # 每次转向增加能耗
        return length + turn_penalty

    def _estimate_energy_3d(self, points: List[Dict], turn_count: int) -> float:
        """
        估算 3D 路径能耗
        基础能耗 + 转向惩罚 + 高度变化惩罚
        """
        length = self._calculate_length_3d(points)

        # 计算高度变化
        z_diff = abs(points[-1]["z"] - points[0]["z"])
        height_penalty = z_diff * 1.5  # 高度变化消耗更多能量

        turn_penalty = turn_count * 0.8

        return length + height_penalty + turn_penalty

    def _length_score(self, length: float) -> float:
        """
        计算长度得分 (越短越好)

        Args:
            length: 路径总长度

        Returns:
            得分 (0-1)
        """
        score = 1 - (length / self.max_possible_length)
        return max(0, min(1, score))

    def _smoothness_score(self, turns: int, num_points: int) -> float:
        """
        计算平滑度得分 (转向越少越好)

        Args:
            turns: 转向次数
            num_points: 路径点数

        Returns:
            得分 (0-1)
        """
        max_turns = max(0, num_points - 1)
        if max_turns == 0:
            return 1.0

        score = 1 - (turns / max_turns)
        return max(0, score)

    def _energy_score(self, energy: float) -> float:
        """
        计算能耗得分 (越低越好)

        Args:
            energy: 估算能耗

        Returns:
            得分 (0-1)
        """
        max_energy = self.max_possible_length * 1.5
        score = 1 - (energy / max_energy)
        return max(0, min(1, score))


def print_evaluation_result(result: Dict, device: str = "Dog") -> None:
    """打印路径评估结果"""
    print(f"\n{'='*50}")
    print(f"    {device} 路径评估结果")
    print(f"{'='*50}")
    print(f"\n路径评分:")
    print(f"  - 总分: {result['total_score']:.3f}")
    print(f"  - 长度得分: {result['length_score']:.3f}")
    print(f"  - 平滑度得分: {result['smoothness_score']:.3f}")
    print(f"  - 能耗得分: {result['energy_score']:.3f}")
    print(f"\n路径统计:")
    print(f"  - 总长度: {result['path_length']:.2f}m")
    print(f"  - 转向次数: {result['turn_count']}")
    print(f"  - 预计时间: {result['estimated_time']:.2f}s")
    print(f"{'='*50}")


def main():
    """测试路径评估"""
    # 创建测试路径
    test_path_2d = {
        "points": [
            {"x": 0, "y": 0},
            {"x": 5, "y": 0},
            {"x": 5, "y": 5},
            {"x": 10, "y": 5},
            {"x": 10, "y": 10},
            {"x": 40, "y": 40}
        ]
    }

    test_path_3d = {
        "points": [
            {"x": 0, "y": 0, "z": 0},
            {"x": 5, "y": 0, "z": 2},
            {"x": 5, "y": 5, "z": 4},
            {"x": 10, "y": 5, "z": 6},
            {"x": 10, "y": 10, "z": 8},
            {"x": 40, "y": 40, "z": 10}
        ]
    }

    evaluator = PathEvaluator()

    # 评估 2D 路径
    print("2D 路径评估:")
    result_2d = evaluator.evaluate_2d(test_path_2d)
    print_evaluation_result(result_2d, "Dog")

    # 评估 3D 路径
    print("\n3D 路径评估:")
    result_3d = evaluator.evaluate_3d(test_path_3d)
    print_evaluation_result(result_3d, "UAV")


if __name__ == "__main__":
    main()
