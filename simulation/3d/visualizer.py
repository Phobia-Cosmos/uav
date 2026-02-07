#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualizer

路径可视化模块。
支持 2D 和 3D 路径显示。
"""

import os
from typing import Dict, List, Any
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class PathVisualizer:
    """路径可视化器"""

    def __init__(self, map_data: Dict):
        """
        初始化可视化器

        Args:
            map_data: 地图数据字典
        """
        self.map_data = map_data
        self.map_size = (
            map_data.get("size", {}).get("x", 50),
            map_data.get("size", {}).get("y", 50),
            map_data.get("size", {}).get("z", 30)
        )

    def visualize_dog_2d(self, path_data: Dict,
                        output_path: str = None,
                        show: bool = True) -> None:
        """
        可视化机器狗 2D 路径

        Args:
            path_data: 2D 路径数据
            output_path: 输出文件路径
            show: 是否显示图像
        """
        fig, ax = plt.subplots(figsize=(10, 10))

        # 绘制障碍物
        obstacles = self.map_data.get("dog_obstacles", [])
        for obs in obstacles:
            if obs.get("type") == "rectangle":
                x = obs.get("position", {}).get("x", 0)
                y = obs.get("position", {}).get("y", 0)
                width = obs.get("size", {}).get("width", 1)
                height = obs.get("size", {}).get("height", 1)
                rect = plt.Rectangle(
                    (x - width/2, y - height/2),
                    width, height,
                    facecolor='lightgray',
                    edgecolor='black',
                    linewidth=2
                )
                ax.add_patch(rect)

            elif obs.get("type") == "circle":
                x = obs.get("position", {}).get("x", 0)
                y = obs.get("position", {}).get("y", 0)
                radius = obs.get("size", {}).get("radius", 1)
                circle = plt.Circle(
                    (x, y),
                    radius,
                    facecolor='lightgray',
                    edgecolor='black',
                    linewidth=2
                )
                ax.add_patch(circle)

        # 绘制起点和终点
        start = self.map_data.get("start", {"x": 0, "y": 0})
        goal = self.map_data.get("goal", {"x": 40, "y": 40})

        ax.scatter(start["x"], start["y"],
                   c='green', s=300, marker='s',
                   label='Start', zorder=5, edgecolors='darkgreen', linewidth=2)
        ax.scatter(goal["x"], goal["y"],
                   c='red', s=300, marker='*',
                   label='Goal', zorder=5, edgecolors='darkred', linewidth=2)

        # 绘制路径
        if path_data and "points" in path_data:
            points = path_data["points"]
            px = [p["x"] for p in points]
            py = [p["y"] for p in points]

            ax.plot(px, py, 'orange', linewidth=3, label='Dog Path', alpha=0.9)
            ax.scatter(px, py, c='orange', s=50, zorder=4, edgecolors='darkorange', linewidth=0.5)

        # 计算评分
        from path_evaluator import PathEvaluator
        evaluator = PathEvaluator()
        result = evaluator.evaluate_2d(path_data) if path_data else {}

        # 设置标题和标签
        title = f"Dog 2D Path Planning (A*)\n"
        if result:
            title += f"Length: {result.get('path_length', 0):.1f}m | "
            title += f"Score: {result.get('total_score', 0):.3f}"

        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.set_xlabel("X (meters)", fontsize=12)
        ax.set_ylabel("Y (meters)", fontsize=12)
        ax.legend(loc='upper right', fontsize=10)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_aspect('equal')
        ax.set_xlim(-2, self.map_size[0] + 2)
        ax.set_ylim(-2, self.map_size[1] + 2)

        # 添加比例尺
        scale_length = 10
        ax.plot([5, 5 + scale_length], [5, 5], 'k-', linewidth=3)
        ax.text(5 + scale_length/2, 3, f'{scale_length}m', ha='center', fontsize=10)

        plt.tight_layout()

        if output_path:
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"2D 路径图已保存到: {output_path}")

        if show:
            plt.show()

        plt.close()

    def visualize_uav_3d(self, path_data: Dict,
                         output_path: str = None,
                         show: bool = True) -> None:
        """
        可视化无人机 3D 路径

        Args:
            path_data: 3D 路径数据
            output_path: 输出文件路径
            show: 是否显示图像
        """
        fig = plt.figure(figsize=(16, 6))

        # 3D 视图
        ax1 = fig.add_subplot(121, projection='3d')

        # 绘制障碍物
        obstacles = self.map_data.get("uav_obstacles", [])
        for obs in obstacles:
            if obs.get("type") == "box":
                x = obs.get("position", {}).get("x", 0)
                y = obs.get("position", {}).get("y", 0)
                z = obs.get("position", {}).get("z", 0)
                l = obs.get("size", {}).get("length", 1)
                w = obs.get("size", {}).get("width", 1)
                h = obs.get("size", {}).get("height", 1)

                ax1.bar3d(x - l/2, y - w/2, z - h/2,
                          l, w, h,
                          color='lightgray',
                          edgecolor='black',
                          alpha=0.8)

        # 绘制起点和终点
        start = self.map_data.get("start", {"x": 0, "y": 0, "z": 0})
        goal = self.map_data.get("goal", {"x": 40, "y": 40, "z": 10})

        ax1.scatter(start["x"], start["y"], start["z"],
                    c='green', s=300, marker='s',
                    label='Start', depthshade=True)
        ax1.scatter(goal["x"], goal["y"], goal["z"],
                    c='red', s=300, marker='*',
                    label='Goal', depthshade=True)

        # 绘制路径
        if path_data and "points" in path_data:
            points = path_data["points"]
            px = [p["x"] for p in points]
            py = [p["y"] for p in points]
            pz = [p["z"] for p in points]

            ax1.plot(px, py, pz, 'blue', linewidth=3, label='UAV Path', alpha=0.9)
            ax1.scatter(px, py, pz, c='blue', s=30, depthshade=True)

        ax1.set_title("UAV 3D Path", fontsize=14, fontweight='bold')
        ax1.set_xlabel("X (m)", fontsize=10)
        ax1.set_ylabel("Y (m)", fontsize=10)
        ax1.set_zlabel("Z (m)", fontsize=10)
        ax1.legend(loc='upper right')

        # 2D 俯视图
        ax2 = fig.add_subplot(122)

        for obs in obstacles:
            if obs.get("type") == "box":
                x = obs.get("position", {}).get("x", 0)
                y = obs.get("position", {}).get("y", 0)
                l = obs.get("size", {}).get("length", 1)
                w = obs.get("size", {}).get("width", 1)

                rect = plt.Rectangle(
                    (x - l/2, y - w/2),
                    l, w,
                    facecolor='lightgray',
                    edgecolor='black'
                )
                ax2.add_patch(rect)

        ax2.scatter(start["x"], start["y"],
                     c='green', s=300, marker='s', label='Start')
        ax2.scatter(goal["x"], goal["y"],
                     c='red', s=300, marker='*', label='Goal')

        if path_data and "points" in path_data:
            points = path_data["points"]
            px = [p["x"] for p in points]
            py = [p["y"] for p in points]

            ax2.plot(px, py, 'blue', linewidth=3, label='UAV Path')

        ax2.set_title("Top View", fontsize=14, fontweight='bold')
        ax2.set_xlabel("X (m)", fontsize=10)
        ax2.set_ylabel("Y (m)", fontsize=10)
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3, linestyle='--')
        ax2.set_aspect('equal')
        ax2.set_xlim(-2, self.map_size[0] + 2)
        ax2.set_ylim(-2, self.map_size[1] + 2)

        # 计算评分
        from path_evaluator import PathEvaluator
        evaluator = PathEvaluator()
        result = evaluator.evaluate_3d(path_data) if path_data else {}

        # 添加总标题
        title = f"UAV 3D Path Planning (A*)"
        if result:
            title += f"\nLength: {result.get('path_length', 0):.1f}m | Score: {result.get('total_score', 0):.3f}"

        fig.suptitle(title, fontsize=14, fontweight='bold', y=1.02)

        plt.tight_layout()

        if output_path:
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"3D 路径图已保存到: {output_path}")

        if show:
            plt.show()

        plt.close()

    def visualize_both(self, dog_path: Dict, uav_path: Dict,
                      output_path: str = None,
                      show: bool = True) -> None:
        """
        同时可视化机器狗和无人机路径

        Args:
            dog_path: 机器狗路径
            uav_path: 无人机路径
            output_path: 输出文件路径
            show: 是否显示图像
        """
        fig = plt.figure(figsize=(18, 8))

        # 左侧: 机器狗 2D
        ax1 = fig.add_subplot(121)

        obstacles = self.map_data.get("dog_obstacles", [])
        for obs in obstacles:
            if obs.get("type") == "rectangle":
                x = obs.get("position", {}).get("x", 0)
                y = obs.get("position", {}).get("y", 0)
                w = obs.get("size", {}).get("width", 1)
                h = obs.get("size", {}).get("height", 1)
                rect = plt.Rectangle((x - w/2, y - h/2), w, h,
                                     facecolor='lightgray', edgecolor='black')
                ax1.add_patch(rect)
            elif obs.get("type") == "circle":
                x = obs.get("position", {}).get("x", 0)
                y = obs.get("position", {}).get("y", 0)
                r = obs.get("size", {}).get("radius", 1)
                ax1.add_patch(plt.Circle((x, y), r, facecolor='lightgray', edgecolor='black'))

        start = self.map_data.get("start", {"x": 0, "y": 0})
        goal = self.map_data.get("goal", {"x": 40, "y": 40})

        ax1.scatter(start["x"], start["y"], c='green', s=300, marker='s', label='Start')
        ax1.scatter(goal["x"], goal["y"], c='red', s=300, marker='*', label='Goal')

        if dog_path and "points" in dog_path:
            px = [p["x"] for p in dog_path["points"]]
            py = [p["y"] for p in dog_path["points"]]
            ax1.plot(px, py, 'orange', linewidth=3, label='Dog Path')

        ax1.set_title("Dog 2D Path", fontsize=14, fontweight='bold')
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')

        # 右侧: 无人机 3D
        ax2 = fig.add_subplot(122, projection='3d')

        obstacles_3d = self.map_data.get("uav_obstacles", [])
        for obs in obstacles_3d:
            if obs.get("type") == "box":
                x = obs.get("position", {}).get("x", 0)
                y = obs.get("position", {}).get("y", 0)
                z = obs.get("position", {}).get("z", 0)
                l = obs.get("size", {}).get("length", 1)
                w = obs.get("size", {}).get("width", 1)
                h = obs.get("size", {}).get("height", 1)
                ax2.bar3d(x - l/2, y - w/2, z - h/2, l, w, h,
                          color='lightgray', edgecolor='black', alpha=0.8)

        ax2.scatter(start["x"], start["y"], start["z"],
                   c='green', s=300, marker='s', label='Start', depthshade=True)
        ax2.scatter(goal["x"], goal["y"], goal["z"],
                   c='red', s=300, marker='*', label='Goal', depthshade=True)

        if uav_path and "points" in uav_path:
            px = [p["x"] for p in uav_path["points"]]
            py = [p["y"] for p in uav_path["points"]]
            pz = [p["z"] for p in uav_path["points"]]
            ax2.plot(px, py, pz, 'blue', linewidth=3, label='UAV Path')

        ax2.set_title("UAV 3D Path", fontsize=14, fontweight='bold')
        ax2.set_xlabel("X (m)")
        ax2.set_ylabel("Y (m)")
        ax2.set_zlabel("Z (m)")
        ax2.legend()

        fig.suptitle("Path Planning Results", fontsize=16, fontweight='bold', y=1.02)

        plt.tight_layout()

        if output_path:
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"组合路径图已保存到: {output_path}")

        if show:
            plt.show()

        plt.close()


def main():
    """测试可视化"""
    from map_generator import load as load_map

    # 加载地图
    config_path = os.path.join(os.path.dirname(__file__), "config/scenario_01.json")
    map_data = load_map(config_path)

    # 创建可视化器
    visualizer = PathVisualizer(map_data)

    # 加载路径数据 (模拟)
    dog_path = {
        "points": [
            {"x": 0, "y": 0},
            {"x": 5, "y": 2},
            {"x": 10, "y": 8},
            {"x": 15, "y": 12},
            {"x": 25, "y": 18},
            {"x": 35, "y": 30},
            {"x": 40, "y": 40}
        ]
    }

    uav_path = {
        "points": [
            {"x": 0, "y": 0, "z": 0},
            {"x": 8, "y": 3, "z": 3},
            {"x": 15, "y": 8, "z": 5},
            {"x": 22, "y": 15, "z": 7},
            {"x": 30, "y": 25, "z": 9},
            {"x": 40, "y": 40, "z": 10}
        ]
    }

    # 可视化
    os.makedirs(os.path.join(os.path.dirname(__file__), "output"), exist_ok=True)

    print("生成 2D 路径图...")
    visualizer.visualize_dog_2d(
        dog_path,
        output_path=os.path.join(os.path.dirname(__file__), "output/dog_2d.png"),
        show=False
    )

    print("生成 3D 路径图...")
    visualizer.visualize_uav_3d(
        uav_path,
        output_path=os.path.join(os.path.dirname(__file__), "output/uav_3d.png"),
        show=False
    )

    print("生成组合路径图...")
    visualizer.visualize_both(
        dog_path, uav_path,
        output_path=os.path.join(os.path.dirname(__file__), "output/both_paths.png"),
        show=True
    )


if __name__ == "__main__":
    main()
