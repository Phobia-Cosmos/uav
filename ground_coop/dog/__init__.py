"""
Ground-Air Cooperation System - Dog (Machine Dog) Module

机器狗端模块，监听无人机命令并控制运动。
"""

from .motion_adapter import MotionAdapter, DogPosition

__all__ = ['MotionAdapter', 'DogPosition']
