#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main Program for UAV 3D Path Planning

无人机 3D 路径规划主程序。
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from map_generator import load as load_map
from a_star_3d import AStar3D
from path_evaluator import PathEvaluator, print_evaluation_result
from visualizer import PathVisualizer


def main():
    """主函数"""
    # 加载地图配置
    config_path = os.path.join(os.path.dirname(__file__), "config/scenario_01.json")

    print("=" * 60)
    print("    无人机 3D 路径规划 (A*)")
    print("=" * 60)

    try:
        map_data = load_map(config_path)
    except FileNotFoundError:
        print(f"\n错误: 找不到配置文件 {config_path}")
        print("请先运行 map_generator.py 生成默认场景")
        return

    print(f"\n场景: {map_data['name']}")
    print(f"描述: {map_data.get('description', '无')}")
    print(f"地图尺寸: {map_data['size']['x']}m x {map_data['size']['y']}m x {map_data['size']['z']}m")
    print(f"障碍物数量: {len(map_data['uav_obstacles'])}")

    # 提取起点和终点
    start = (
        map_data['start']['x'],
        map_data['start']['y'],
        map_data['start']['z']
    )
    goal = (
        map_data['goal']['x'],
        map_data['goal']['y'],
        map_data['goal']['z']
    )

    print(f"\n起点: ({start[0]}, {start[1]}, {start[2]})")
    print(f"终点: ({goal[0]}, {goal[1]}, {goal[2]})")

    # 创建规划器
    grid_size = (
        map_data['size']['x'],
        map_data['size']['y'],
        map_data['size']['z']
    )
    planner = AStar3D(
        obstacles=map_data['uav_obstacles'],
        grid_size=grid_size
    )

    # 执行规划
    print(f"\n[运行 A* 算法...]")

    path_result = planner.plan(start, goal, inflate_radius=0)

    if path_result:
        # 评估路径
        evaluator = PathEvaluator()
        score_result = evaluator.evaluate_3d(path_result)

        # 打印评估结果
        print_evaluation_result(score_result, "UAV")

        # 可视化
        print("\n[生成可视化...]")
        visualizer = PathVisualizer(map_data)
        visualizer.visualize_uav_3d(
            path_result,
            output_path=os.path.join(os.path.dirname(__file__), "output/uav_path_3d.png"),
            show=True
        )

    else:
        print("\n路径规划失败!")
        print("可能原因:")
        print("  1. 起点或终点在障碍物内")
        print("  2. 障碍物阻挡了所有路径")
        print("  3. 请尝试调整地图配置")


if __name__ == "__main__":
    os.makedirs(os.path.join(os.path.dirname(__file__), "output"), exist_ok=True)
    main()
