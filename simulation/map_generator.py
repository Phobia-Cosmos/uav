#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Map Generator

生成用于路径规划的地图和障碍物。
支持 2D (机器狗) 和 3D (无人机) 障碍物定义。
"""

import json
import random
import math
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass, asdict, field


@dataclass
class Point2D:
    """2D 坐标点"""
    x: float
    y: float

    def to_dict(self) -> Dict:
        return {"x": self.x, "y": self.y}

    @classmethod
    def from_dict(cls, data: Dict) -> 'Point2D':
        return cls(x=data["x"], y=data["y"])


@dataclass
class Point3D:
    """3D 坐标点"""
    x: float
    y: float
    z: float

    def to_dict(self) -> Dict:
        return {"x": self.x, "y": self.y, "z": self.z}

    @classmethod
    def from_dict(cls, data: Dict) -> 'Point3D':
        return cls(x=data["x"], y=data["y"], z=data["z"])


@dataclass
class Obstacle2D:
    """2D 障碍物 (机器狗)"""
    id: str
    type: str  # "rectangle", "circle", "wall", or "polygon"
    position: Point2D
    size: Dict[str, float]  # {"width": x, "height": y} 或 {"radius": r} 或 {"width": w}
    end_point: Optional[Point2D] = field(default=None)
    vertices: List[Point2D] = field(default_factory=list)
    uav_passable: bool = field(default=False)
    semantic_type: str = field(default="generic")
    properties: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict:
        result = {
            "id": self.id,
            "type": self.type,
            "position": self.position.to_dict(),
            "size": self.size,
        }
        if self.end_point:
            result["end_point"] = self.end_point.to_dict()
        if self.vertices:
            result["vertices"] = [vertex.to_dict() for vertex in self.vertices]
        if self.uav_passable:
            result["uav_passable"] = True
        if self.semantic_type != "generic":
            result["semantic_type"] = self.semantic_type
        if self.properties:
            result["properties"] = self.properties
        return result

    @classmethod
    def from_dict(cls, data: Dict) -> 'Obstacle2D':
        if data["type"] == "wall":
            position = Point2D.from_dict(data["start_point"])
            end_point = Point2D.from_dict(data["end_point"])
            size = {"width": data.get("width", 1)}
            vertices = []
        elif data["type"] == "polygon":
            vertices = [Point2D.from_dict(vertex) for vertex in data["vertices"]]
            position = cls._polygon_centroid(vertices)
            end_point = None
            size = data.get("size", {})
        else:
            position = Point2D.from_dict(data["position"])
            end_point = None
            size = data["size"]
            vertices = []
        return cls(
            id=data["id"],
            type=data["type"],
            position=position,
            size=size,
            end_point=end_point,
            vertices=vertices,
            uav_passable=data.get("uav_passable", False),
            semantic_type=data.get("semantic_type", "generic"),
            properties=data.get("properties", {})
        )

    @staticmethod
    def _polygon_centroid(vertices: List['Point2D']) -> 'Point2D':
        if not vertices:
            return Point2D(0.0, 0.0)
        x = sum(vertex.x for vertex in vertices) / len(vertices)
        y = sum(vertex.y for vertex in vertices) / len(vertices)
        return Point2D(x=x, y=y)

    def contains(self, point: Tuple[float, float]) -> bool:
        """检查点是否在障碍物内"""
        px, py = point
        ox, oy = self.position.x, self.position.y

        if self.type == "rectangle":
            width = self.size["width"]
            height = self.size["height"]
            return (ox - width/2 <= px <= ox + width/2 and
                    oy - height/2 <= py <= oy + height/2)
        elif self.type == "circle":
            radius = self.size["radius"]
            return ((px - ox)**2 + (py - oy)**2 <= radius**2)
        elif self.type == "wall":
            if self.end_point is None:
                return False
            return self._point_to_segment_distance(px, py) <= self.size.get("width", 1) / 2
        elif self.type == "polygon":
            return self._point_in_polygon(px, py)
        return False

    def distance_to_point(self, point: Tuple[float, float]) -> float:
        """计算点到障碍物占据区域的最短距离。"""
        px, py = point
        ox, oy = self.position.x, self.position.y

        if self.type == "rectangle":
            width = self.size["width"]
            height = self.size["height"]
            dx = max(abs(px - ox) - width / 2, 0)
            dy = max(abs(py - oy) - height / 2, 0)
            return math.sqrt(dx * dx + dy * dy)

        if self.type == "circle":
            radius = self.size["radius"]
            center_distance = math.sqrt((px - ox) ** 2 + (py - oy) ** 2)
            return max(0.0, center_distance - radius)

        if self.type == "wall":
            wall_distance = self._point_to_segment_distance(px, py)
            return max(0.0, wall_distance - self.size.get("width", 1) / 2)

        if self.type == "polygon":
            if self._point_in_polygon(px, py):
                return 0.0
            return self._distance_to_polygon_edges(px, py)

        return float('inf')

    def _point_to_segment_distance(self, px: float, py: float) -> float:
        """计算点到线段的距离"""
        if self.end_point is None:
            return float('inf')
        x1 = self.position.x
        y1 = self.position.y
        x2 = self.end_point.x
        y2 = self.end_point.y
        
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx * dx + dy * dy
        
        if length_sq == 0:
            return math.sqrt((px - x1)**2 + (py - y1)**2)
        
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / length_sq))
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        return math.sqrt((px - closest_x)**2 + (py - closest_y)**2)

    def _point_in_polygon(self, px: float, py: float) -> bool:
        if len(self.vertices) < 3:
            return False

        inside = False
        j = len(self.vertices) - 1
        for i, vertex in enumerate(self.vertices):
            prev = self.vertices[j]
            intersects = ((vertex.y > py) != (prev.y > py)) and (
                px < (prev.x - vertex.x) * (py - vertex.y) / ((prev.y - vertex.y) or 1e-9) + vertex.x
            )
            if intersects:
                inside = not inside
            j = i
        return inside

    def _distance_to_polygon_edges(self, px: float, py: float) -> float:
        if len(self.vertices) < 2:
            return float('inf')

        min_distance = float('inf')
        for index, start in enumerate(self.vertices):
            end = self.vertices[(index + 1) % len(self.vertices)]
            distance = self._point_to_custom_segment_distance(px, py, start.x, start.y, end.x, end.y)
            min_distance = min(min_distance, distance)
        return min_distance

    @staticmethod
    def _point_to_custom_segment_distance(px: float, py: float,
                                          x1: float, y1: float,
                                          x2: float, y2: float) -> float:
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx * dx + dy * dy

        if length_sq == 0:
            return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / length_sq))
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)


@dataclass
class Obstacle3D:
    """3D 障碍物 (无人机)"""
    id: str
    type: str  # "box"
    position: Point3D
    size: Dict[str, float]  # {"length": x, "width": y, "height": z}

    def to_dict(self) -> Dict:
        return {
            "id": self.id,
            "type": self.type,
            "position": self.position.to_dict(),
            "size": self.size
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'Obstacle3D':
        return cls(
            id=data["id"],
            type=data["type"],
            position=Point3D.from_dict(data["position"]),
            size=data["size"]
        )

    def contains(self, point: Tuple[float, float, float]) -> bool:
        """检查点是否在障碍物内"""
        px, py, pz = point
        ox, oy, oz = self.position.x, self.position.y, self.position.z
        length = self.size["length"]
        width = self.size["width"]
        height = self.size["height"]

        return (ox - length/2 <= px <= ox + length/2 and
                oy - width/2 <= py <= oy + width/2 and
                oz - height/2 <= pz <= oz + height/2)


class MapGenerator:
    """地图生成器"""

    def __init__(self, size: Tuple[int, int, int] = (50, 50, 30)):
        """
        初始化地图生成器

        Args:
            size: (x, y, z) 地图尺寸
        """
        self.size = size  # (width, height, max_z)
        self.dog_obstacles: List[Obstacle2D] = []
        self.uav_obstacles: List[Obstacle3D] = []

    def add_dog_obstacle(self, id: str, obs_type: str,
                         position: Tuple[float, float],
                         size: Dict[str, float]) -> None:
        """添加机器狗障碍物"""
        obstacle = Obstacle2D(
            id=id,
            type=obs_type,
            position=Point2D(x=position[0], y=position[1]),
            size=size
        )
        self.dog_obstacles.append(obstacle)

    def add_uav_obstacle(self, id: str,
                         position: Tuple[float, float, float],
                         size: Dict[str, float]) -> None:
        """添加无人机障碍物"""
        obstacle = Obstacle3D(
            id=id,
            type="box",
            position=Point3D(x=position[0], y=position[1], z=position[2]),
            size=size
        )
        self.uav_obstacles.append(obstacle)

    def generate_random_obstacles(self,
                                   num_dog_obs: int = 5,
                                   num_uav_obs: int = 3) -> None:
        """生成随机障碍物"""
        # 机器狗 2D 障碍物
        for i in range(num_dog_obs):
            x = random.uniform(5, self.size[0] - 10)
            y = random.uniform(5, self.size[1] - 10)
            obs_type = random.choice(["rectangle", "circle"])

            if obs_type == "rectangle":
                width = random.uniform(2, 8)
                height = random.uniform(2, 8)
                size = {"width": width, "height": height}
            else:
                radius = random.uniform(1, 3)
                size = {"radius": radius}

            self.add_dog_obstacle(f"obs_dog_{i+1}", obs_type, (x, y), size)

        # 无人机 3D 障碍物
        for i in range(num_uav_obs):
            x = random.uniform(10, self.size[0] - 15)
            y = random.uniform(10, self.size[1] - 15)
            z = random.uniform(5, self.size[2] - 10)
            size = {
                "length": random.uniform(3, 8),
                "width": random.uniform(3, 8),
                "height": random.uniform(5, 15)
            }
            self.add_uav_obstacle(f"obs_uav_{i+1}", (x, y, z), size)

    def to_dict(self, name: str = "Generated Map",
                description: str = "随机生成的地图") -> Dict:
        """导出为字典"""
        return {
            "name": name,
            "description": description,
            "size": {
                "x": self.size[0],
                "y": self.size[1],
                "z": self.size[2]
            },
            "start": {"x": 0, "y": 0, "z": 0},
            "goal": {"x": self.size[0] - 5, "y": self.size[1] - 5, "z": self.size[2] // 3},
            "dog_obstacles": [obs.to_dict() for obs in self.dog_obstacles],
            "uav_obstacles": [obs.to_dict() for obs in self.uav_obstacles]
        }

    def save(self, filepath: str,
             name: str = "Generated Map",
             description: str = "随机生成的地图") -> None:
        """保存为 JSON 文件"""
        data = self.to_dict(name, description)
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    def is_collision_2d(self, point: Tuple[float, float]) -> bool:
        """检查 2D 点是否与障碍物碰撞"""
        for obs in self.dog_obstacles:
            if obs.contains(point):
                return True
        return False

    def is_collision_3d(self, point: Tuple[float, float, float]) -> bool:
        """检查 3D 点是否与障碍物碰撞"""
        for obs in self.uav_obstacles:
            if obs.contains(point):
                return True
        return False


def load(filepath: str) -> Dict:
    """从 JSON 文件加载地图"""
    with open(filepath, 'r', encoding='utf-8') as f:
        return json.load(f)


def create_default_scenario() -> Dict:
    """创建默认场景配置"""
    return {
        "name": "搜救场景_01",
        "description": "简单搜救场景，包含少量障碍物",
        "size": {
            "x": 50,
            "y": 50,
            "z": 30
        },
        "start": {
            "x": 0,
            "y": 0,
            "z": 0
        },
        "goal": {
            "x": 40,
            "y": 40,
            "z": 10
        },
        "dog_obstacles": [
            {
                "id": "obs_001",
                "type": "rectangle",
                "position": {"x": 15, "y": 10},
                "size": {"width": 8, "height": 5}
            },
            {
                "id": "obs_002",
                "type": "circle",
                "position": {"x": 30, "y": 25},
                "size": {"radius": 3}
            }
        ],
        "uav_obstacles": [
            {
                "id": "obs_003",
                "type": "box",
                "position": {"x": 20, "y": 15, "z": 5},
                "size": {"length": 6, "width": 4, "height": 15}
            }
        ]
    }


def save_default_scenario(filepath: str) -> None:
    """保存默认场景配置"""
    data = create_default_scenario()
    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


if __name__ == "__main__":
    import os

    # 创建默认场景
    config_dir = os.path.join(os.path.dirname(__file__), "config")
    os.makedirs(config_dir, exist_ok=True)

    filepath = os.path.join(config_dir, "scenario_01.json")
    save_default_scenario(filepath)
    print(f"默认场景已保存到: {filepath}")

    # 测试加载
    config = load(filepath)
    print(f"\n场景名称: {config['name']}")
    print(f"地图尺寸: {config['size']['x']} x {config['size']['y']} x {config['size']['z']}")
    print(f"起点: {config['start']}")
    print(f"终点: {config['goal']}")
    print(f"机器狗障碍物: {len(config['dog_obstacles'])} 个")
    print(f"无人机障碍物: {len(config['uav_obstacles'])} 个")
