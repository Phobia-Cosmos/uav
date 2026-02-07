# 阶段三：设备端独立规划

## 1. 概述

本阶段目标：
- 机器狗端实现独立的 2D 路径规划
- 无人机端实现独立的 3D 路径规划
- 路径数据格式标准化
- 实现实时重规划机制

## 2. 架构设计

```
┌─────────────────────────────────────────────────────────┐
│                   设备端规划架构                          │
├─────────────────────────────────────────────────────────┤
│                                                         │
│   ┌─────────────────────────────────────────────────┐   │
│   │              机器狗系统                          │   │
│   │  ┌──────────┐  ┌──────────┐  ┌──────────┐    │   │
│   │  │ 传感器    │→ │ 规划器   │→ │ 执行器   │    │   │
│   │  │          │  │ (A*)     │  │          │    │   │
│   │  └──────────┘  └──────────┘  └──────────┘    │   │
│   │       │              │              │         │   │
│   │       ↓              ↓              ↓         │   │
│   │  ┌──────────────────────────────────────┐     │   │
│   │  │         路径缓存管理器                  │     │   │
│   │  └──────────────────────────────────────┘     │   │
│   │                      │                      │   │
│   │                      ↓                      │   │
│   │               ┌──────────────┐             │   │
│   │               │  TCP 客户端  │─────────────┼───┘   │
│   │               └──────────────┘              │       │
│   └──────────────────────────────────────────────┘       │
│                                                         │
│   ┌─────────────────────────────────────────────────┐   │
│   │              无人机系统                          │   │
│   │  ┌──────────┐  ┌──────────┐  ┌──────────┐    │   │
│   │  │ 传感器    │→ │ 规划器   │→ │ 执行器   │    │   │
│   │  │          │  │ (A*)     │  │          │    │   │
│   │  └──────────┘  └──────────┘  └──────────┘    │   │
│   │       │              │              │         │   │
│   │       ↓              ↓              ↓         │   │
│   │  ┌──────────────────────────────────────┐     │   │
│   │  │         路径缓存管理器                  │     │   │
│   │  └──────────────────────────────────────┘     │   │
│   │                      │                      │   │
│   │                      ↓                      │   │
│   │               ┌──────────────┐             │   │
│   │               │ TCP 服务端   │←────────────┼───┘   │
│   │               └──────────────┘              │       │
│   └──────────────────────────────────────────────┘       │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

## 3. 路径数据格式标准化

### 3.1 统一路径格式

```python
# unified_path.py

from dataclasses import dataclass, field
from typing import List, Dict, Optional
from enum import Enum
import time
import uuid


class PathType(Enum):
    """路径类型"""
    DOG_2D = "dog_2d"
    UAV_3D = "uav_3d"


class PathStatus(Enum):
    """路径状态"""
    PENDING = "pending"
    ACTIVE = "active"
    COMPLETED = "completed"
    CANCELLED = "cancelled"


@dataclass
class Point:
    """路径点"""
    x: float
    y: float
    z: float = 0.0

    def distance_to(self, other: 'Point') -> float:
        """计算到另一个点的距离"""
        return ((self.x - other.x)**2 +
                (self.y - other.y)**2 +
                (self.z - other.z)**2) ** 0.5

    def to_dict(self) -> Dict:
        return {"x": self.x, "y": self.y, "z": self.z}

    @classmethod
    def from_dict(cls, data: Dict) -> 'Point':
        return cls(x=data["x"], y=data["y"], z=data.get("z", 0))


@dataclass
class PathMetadata:
    """路径元数据"""
    path_id: str
    path_type: PathType
    status: PathStatus
    created_at: float
    updated_at: float
    from_position: Point
    to_position: Point
    total_length: float
    point_count: int
    algorithm: str
    score: Dict[str, float]
    execution_progress: float = 0.0  # 执行进度 0-1
    estimated_time: float = 0.0  # 预计时间(秒)


@dataclass
class UnifiedPath:
    """统一路径格式"""
    metadata: PathMetadata
    points: List[Point]

    def to_dict(self) -> Dict:
        """转换为字典"""
        return {
            "version": "1.0",
            "metadata": {
                "path_id": self.metadata.path_id,
                "path_type": self.metadata.path_type.value,
                "status": self.metadata.status.value,
                "created_at": self.metadata.created_at,
                "updated_at": self.metadata.updated_at,
                "from_position": self.metadata.from_position.to_dict(),
                "to_position": self.metadata.to_position.to_dict(),
                "total_length": self.metadata.total_length,
                "point_count": self.metadata.point_count,
                "algorithm": self.metadata.algorithm,
                "score": self.metadata.score,
                "execution_progress": self.metadata.execution_progress,
                "estimated_time": self.metadata.estimated_time
            },
            "points": [p.to_dict() for p in self.points]
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'UnifiedPath':
        """从字典创建"""
        meta_data = data["metadata"]
        return cls(
            metadata=PathMetadata(
                path_id=meta_data["path_id"],
                path_type=PathType(meta_data["path_type"]),
                status=PathStatus(meta_data["status"]),
                created_at=meta_data["created_at"],
                updated_at=meta_data["updated_at"],
                from_position=Point.from_dict(meta_data["from_position"]),
                to_position=Point.from_dict(meta_data["to_position"]),
                total_length=meta_data["total_length"],
                point_count=meta_data["point_count"],
                algorithm=meta_data["algorithm"],
                score=meta_data["score"],
                execution_progress=meta_data["execution_progress"],
                estimated_time=meta_data["estimated_time"]
            ),
            points=[Point.from_dict(p) for p in data["points"]]
        )

    def to_json(self) -> str:
        """转换为 JSON 字符串"""
        import json
        return json.dumps(self.to_dict(), indent=2)

    @classmethod
    def from_json(cls, json_str: str) -> 'UnifiedPath':
        """从 JSON 字符串创建"""
        import json
        return cls.from_dict(json.loads(json_str))
```

### 3.2 路径工厂函数

```python
# path_factory.py

from unified_path import UnifiedPath, PathMetadata, Point, PathType, PathStatus
from typing import Dict


class PathFactory:
    """路径工厂"""

    @staticmethod
    def create_dog_path(
            from_pos: Dict,
            to_pos: Dict,
            points: List[Dict],
            score: Dict[str, float],
            algorithm: str = "A*"
    ) -> UnifiedPath:
        """创建机器狗 2D 路径"""
        from_point = Point.from_dict(from_pos)
        to_point = Point.from_dict(to_pos)

        metadata = PathMetadata(
            path_id=f"dog_{int(time.time())}",
            path_type=PathType.DOG_2D,
            status=PathStatus.PENDING,
            created_at=time.time(),
            updated_at=time.time(),
            from_position=from_point,
            to_position=to_point,
            total_length=PathFactory._calc_length(points),
            point_count=len(points),
            algorithm=algorithm,
            score=score
        )

        return UnifiedPath(
            metadata=metadata,
            points=[Point.from_dict(p) for p in points]
        )

    @staticmethod
    def create_uav_path(
            from_pos: Dict,
            to_pos: Dict,
            points: List[Dict],
            score: Dict[str, float],
            algorithm: str = "A*"
    ) -> UnifiedPath:
        """创建无人机 3D 路径"""
        from_point = Point.from_dict(from_pos)
        to_point = Point.from_dict(to_pos)

        metadata = PathMetadata(
            path_id=f"uav_{int(time.time())}",
            path_type=PathType.UAV_3D,
            status=PathStatus.PENDING,
            created_at=time.time(),
            updated_at=time.time(),
            from_position=from_point,
            to_position=to_point,
            total_length=PathFactory._calc_length(points),
            point_count=len(points),
            algorithm=algorithm,
            score=score
        )

        return UnifiedPath(
            metadata=metadata,
            points=[Point.from_dict(p) for p in points]
        )

    @staticmethod
    def _calc_length(points: List[Dict]) -> float:
        """计算路径总长度"""
        if len(points) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(points)):
            p1 = Point.from_dict(points[i-1])
            p2 = Point.from_dict(points[i])
            total += p1.distance_to(p2)
        return total
```

## 4. 路径缓存管理器

```python
# path_cache.py

from unified_path import UnifiedPath, PathStatus
from typing import Dict, Optional, List
import time


class PathCache:
    """路径缓存管理器"""

    def __init__(self, max_cache_size: int = 100):
        self.cache: Dict[str, UnifiedPath] = {}
        self.max_cache_size = max_cache_size

    def add(self, path: UnifiedPath) -> None:
        """添加路径到缓存"""
        path_id = path.metadata.path_id

        # 清理旧路径
        if len(self.cache) >= self.max_cache_size:
            self._evict_oldest()

        self.cache[path_id] = path

    def get(self, path_id: str) -> Optional[UnifiedPath]:
        """获取路径"""
        return self.cache.get(path_id)

    def update_status(self, path_id: str, status: PathStatus) -> bool:
        """更新路径状态"""
        if path_id in self.cache:
            path = self.cache[path_id]
            path.metadata.status = status
            path.metadata.updated_at = time.time()
            return True
        return False

    def get_active_paths(self) -> List[UnifiedPath]:
        """获取所有活跃路径"""
        return [
            p for p in self.cache.values()
            if p.metadata.status == PathStatus.ACTIVE
        ]

    def cancel_path(self, path_id: str) -> bool:
        """取消路径"""
        return self.update_status(path_id, PathStatus.CANCELLED)

    def complete_path(self, path_id: str) -> bool:
        """标记路径完成"""
        return self.update_status(path_id, PathStatus.COMPLETED)

    def clear(self) -> None:
        """清空缓存"""
        self.cache.clear()

    def _evict_oldest(self):
        """清理最旧的路径"""
        if not self.cache:
            return
        oldest_id = min(
            self.cache.keys(),
            key=lambda k: self.cache[k].metadata.created_at
        )
        del self.cache[oldest_id]

    def __len__(self):
        return len(self.cache)

    def get_summary(self) -> Dict:
        """获取缓存摘要"""
        by_status = {}
        for path in self.cache.values():
            status = path.metadata.status.value
            by_status[status] = by_status.get(status, 0) + 1
        return {
            "total": len(self.cache),
            "by_status": by_status
        }
```

## 5. 实时重规划机制

### 5.1 重规划触发条件

```python
# replanner.py

from unified_path import UnifiedPath, PathStatus
from typing import Callable, Optional
import time


class ReplanningTrigger:
    """重规划触发器"""

    def __init__(self,
                 on_trigger: Callable[[str], None] = None,
                 check_interval: float = 1.0):
        self.on_trigger = on_trigger
        self.check_interval = check_interval
        self.triggers = {
            "obstacle_detected": False,
            "path_blocked": False,
            "timeout": False,
            "manual_request": False
        }

    def check_obstacle(self, detected: bool) -> None:
        """检测到障碍物"""
        self.triggers["obstacle_detected"] = detected
        if detected and self.on_trigger:
            self.on_trigger("obstacle")

    def check_path_blocked(self, is_blocked: bool) -> None:
        """路径被阻挡"""
        self.triggers["path_blocked"] = is_blocked
        if is_blocked and self.on_trigger:
            self.on_trigger("blocked")

    def check_timeout(self, elapsed_time: float, max_time: float) -> None:
        """超时检查"""
        self.triggers["timeout"] = elapsed_time > max_time
        if self.triggers["timeout"] and self.on_trigger:
            self.on_trigger("timeout")

    def manual_request(self) -> None:
        """手动触发重规划"""
        self.triggers["manual_request"] = True
        if self.on_trigger:
            self.on_trigger("manual")

    def should_replan(self) -> bool:
        """是否应该重规划"""
        return any(self.triggers.values())

    def reset(self) -> None:
        """重置触发器"""
        self.triggers = {k: False for k in self.triggers}

    def get_trigger_reason(self) -> Optional[str]:
        """获取触发原因"""
        for reason, triggered in self.triggers.items():
            if triggered:
                return reason
        return None
```

### 5.2 重规划执行器

```python
class Replanner:
    """重规划执行器"""

    def __init__(self, planner, cache: PathCache):
        self.planner = planner
        self.cache = cache
        self.trigger = ReplanningTrigger()

    def replan(self, current_position: tuple,
               goal_position: tuple,
               obstacles: list) -> Optional[UnifiedPath]:
        """执行重规划"""
        if not self.trigger.should_replan():
            return None

        reason = self.trigger.get_trigger_reason()
        print(f"[Replanner] 触发重规划: {reason}")

        # 生成新路径
        new_path = self.planner.plan(current_position, goal_position, obstacles)

        if new_path:
            # 更新缓存
            new_path.metadata.status = PathStatus.ACTIVE
            self.cache.add(new_path)

        self.trigger.reset()
        return new_path
```

## 6. 设备端规划器实现

### 6.1 机器狗规划器 (独立运行)

```python
# dog_planner.py

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from a_star_2d import AStar2D
from path_evaluator import PathEvaluator
from unified_path import UnifiedPath
from path_cache import PathCache
from path_factory import PathFactory
import map_generator


class DogPlanner:
    """机器狗路径规划器 (独立运行版本)"""

    def __init__(self, config_path: str):
        # 加载地图
        self.map_data = map_generator.load(config_path)
        self.grid_size = (self.map_data["size"]["x"], self.map_data["size"]["y"])
        self.obstacles = self.map_data["dog_obstacles"]

        # 初始化组件
        self.planner = AStar2D(self.obstacles, self.grid_size)
        self.evaluator = PathEvaluator()
        self.cache = PathCache()

    def plan(self, from_pos: tuple, to_pos: tuple) -> UnifiedPath:
        """规划路径"""
        print(f"[DogPlanner] 规划路径: {from_pos} -> {to_pos}")

        # 执行 A*
        path = self.planner.plan(from_pos, to_pos)

        if path:
            # 评估路径
            score = self.evaluator.evaluate_2d(path)

            # 创建统一格式路径
            unified = PathFactory.create_dog_path(
                from_pos={"x": from_pos[0], "y": from_pos[1]},
                to_pos={"x": to_pos[0], "y": to_pos[1]},
                points=[{"x": p[0], "y": p[1]} for p in path["points"]],
                score=score
            )

            # 添加到缓存
            self.cache.add(unified)

            print(f"[DogPlanner] 路径规划完成: {len(path['points'])} 点, 评分: {score['total']}")
            return unified

        return None

    def get_cached_path(self, path_id: str) -> UnifiedPath:
        """获取缓存路径"""
        return self.cache.get(path_id)

    def get_cache_summary(self) -> dict:
        """获取缓存摘要"""
        return self.cache.get_summary()


def main():
    """主函数"""
    config_path = os.path.join(os.path.dirname(__file__), "config/scenario_01.json")
    planner = DogPlanner(config_path)

    # 规划路径
    start = (planner.map_data["start"]["x"], planner.map_data["start"]["y"])
    goal = (planner.map_data["goal"]["x"], planner.map_data["goal"]["y"])

    path = planner.plan(start, goal)

    if path:
        print(f"\n路径摘要:")
        print(f"  ID: {path.metadata.path_id}")
        print(f"  长度: {path.metadata.total_length:.2f}m")
        print(f"  评分: {path.metadata.score['total']}")
        print(f"  状态: {path.metadata.status.value}")


if __name__ == "__main__":
    main()
```

### 6.2 无人机规划器 (独立运行)

```python
# uav_planner.py

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from a_star_3d import AStar3D
from path_evaluator import PathEvaluator
from unified_path import UnifiedPath
from path_cache import PathCache
from path_factory import PathFactory
import map_generator


class UAVPlanner:
    """无人机路径规划器 (独立运行版本)"""

    def __init__(self, config_path: str):
        # 加载地图
        self.map_data = map_generator.load(config_path)
        self.grid_size = (
            self.map_data["size"]["x"],
            self.map_data["size"]["y"],
            self.map_data["size"]["z"]
        )
        self.obstacles = self.map_data["uav_obstacles"]

        # 初始化组件
        self.planner = AStar3D(self.obstacles, self.grid_size)
        self.evaluator = PathEvaluator()
        self.cache = PathCache()

    def plan(self, from_pos: tuple, to_pos: tuple) -> UnifiedPath:
        """规划路径"""
        print(f"[UAVPlanner] 规划路径: {from_pos} -> {to_pos}")

        # 执行 A*
        path = self.planner.plan(from_pos, to_pos)

        if path:
            # 评估路径
            score = self.evaluator.evaluate_3d(path)

            # 创建统一格式路径
            unified = PathFactory.create_uav_path(
                from_pos={"x": from_pos[0], "y": from_pos[1], "z": from_pos[2]},
                to_pos={"x": to_pos[0], "y": to_pos[1], "z": to_pos[2]},
                points=[{"x": p[0], "y": p[1], "z": p[2]} for p in path["points"]],
                score=score
            )

            # 添加到缓存
            self.cache.add(unified)

            print(f"[UAVPlanner] 路径规划完成: {len(path['points'])} 点, 评分: {score['total']}")
            return unified

        return None

    def get_cache_summary(self) -> dict:
        """获取缓存摘要"""
        return self.cache.get_summary()


def main():
    """主函数"""
    config_path = os.path.join(os.path.dirname(__file__), "config/scenario_01.json")
    planner = UAVPlanner(config_path)

    # 规划路径
    start = (
        planner.map_data["start"]["x"],
        planner.map_data["start"]["y"],
        planner.map_data["start"]["z"]
    )
    goal = (
        planner.map_data["goal"]["x"],
        planner.map_data["goal"]["y"],
        planner.map_data["goal"]["z"]
    )

    path = planner.plan(start, goal)

    if path:
        print(f"\n路径摘要:")
        print(f"  ID: {path.metadata.path_id}")
        print(f"  类型: {path.metadata.path_type.value}")
        print(f"  长度: {path.metadata.total_length:.2f}m")
        print(f"  评分: {path.metadata.score['total']}")


if __name__ == "__main__":
    main()
```

## 7. 运行说明

### 7.1 运行机器狗规划器

```bash
cd simulation_phase3
python dog_planner.py
```

### 7.2 运行无人机规划器

```bash
cd simulation_phase3
python uav_planner.py
```

### 7.3 联合测试

```bash
# 终端 1 - 机器狗
python dog_planner.py

# 终端 2 - 无人机
python uav_planner.py
```

## 8. 与阶段二的差异

| 项目 | 阶段二 | 阶段三 |
|------|--------|--------|
| 路径格式 | 简化的 JSON | 标准化 UnifiedPath |
| 缓存机制 | 无 | PathCache |
| 重规划 | 无 | Replanner |
| 设备独立性 | 模拟器内运行 | 独立运行在设备端 |

## 9. 后续扩展

1. **多目标规划**
   - 支持多个目标点顺序规划
   - 实现 TSP 近似解

2. **动态环境**
   - 实时障碍物更新
   - 在线重规划

3. **性能优化**
   - 路径缓存查询优化
   - 并行规划支持
