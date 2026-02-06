#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motion Adapter for Dog (Machine Dog)

机器狗运动控制适配器，调用ROS服务。

预留接口，实际ROS服务调用由机器狗端同学实现。
"""

import time
import random
from typing import Dict, Optional
from dataclasses import dataclass


@dataclass
class DogPosition:
    """机器狗位置"""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class MotionAdapter:
    """运动控制适配器"""
    
    def __init__(self):
        """初始化运动适配器"""
        self.current_position = DogPosition()
        self._ros_available = False
        
        # 尝试连接ROS服务
        self._init_ros()
    
    def _init_ros(self):
        """初始化ROS连接"""
        try:
            import rospy
            from std_srvs.srv import Trigger, TriggerResponse
            from geometry_msgs.msg import Twist
            
            rospy.init_node('ground_coop_dog', anonymous=True)
            self._ros_available = True
            print("ROS initialized successfully")
            
        except ImportError:
            print("ROS not available, using mock mode")
            self._ros_available = False
        except Exception as e:
            print(f"ROS initialization failed: {e}")
            self._ros_available = False
    
    def move(self, angle: float, duration: float) -> Dict:
        """
        移动命令
        
        Args:
            angle: 移动角度 (0-360度)
            duration: 移动时间 (0-10秒)
            
        Returns:
            {"success": bool, "position": DogPosition}
        """
        # 如果ROS可用，调用ROS服务
        if self._ros_available:
            return self._move_ros(angle, duration)
        
        # 否则使用模拟模式
        return self._move_mock(angle, duration)
    
    def _move_ros(self, angle: float, duration: float) -> Dict:
        """调用ROS服务移动"""
        try:
            import rospy
            from geometry_msgs.msg import Twist
            
            # 创建速度命令
            cmd_vel = Twist()
            
            # 将角度转换为弧度
            angle_rad = angle * 3.14159 / 180.0
            
            # 设置线速度（根据角度分解）
            speed = 0.5  # 基础速度 m/s
            cmd_vel.linear.x = speed * (1 if 0 <= angle < 90 else -1 if 180 <= angle < 270 else 0)
            cmd_vel.linear.y = speed * (1 if 90 <= angle < 180 else -1 if 270 <= angle < 360 else 0)
            cmd_vel.angular.z = 0
            
            # 发布速度命令
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            rospy.sleep(0.1)
            
            # 持续发送速度命令
            rate = rospy.Rate(10)
            start_time = rospy.Time.now()
            
            while rospy.Time.now() - start_time < rospy.Duration(duration):
                pub.publish(cmd_vel)
                rate.sleep()
            
            # 停止
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            pub.publish(cmd_vel)
            
            return {"success": True, "position": self.current_position.__dict__}
            
        except Exception as e:
            return {"success": False, "error": str(e), "position": self.current_position.__dict__}
    
    def _move_mock(self, angle: float, duration: float) -> Dict:
        """
        模拟移动（用于测试）
        
        Args:
            angle: 移动角度
            duration: 移动时间
            
        Returns:
            移动结果
        """
        print(f"[Dog Mock] Moving: angle={angle}°, duration={duration}s")
        
        # 模拟移动
        time.sleep(min(duration, 0.1))  # 快速模拟
        
        # 更新位置
        angle_rad = angle * 3.14159 / 180.0
        distance = 0.5 * duration  # 假设速度0.5m/s
        
        self.current_position.x += distance * (1 if 0 <= angle < 90 else -1 if 180 <= angle < 270 else 0)
        self.current_position.y += distance * (1 if 90 <= angle < 180 else -1 if 270 <= angle < 360 else 0)
        self.current_position.yaw = angle
        
        print(f"[Dog Mock] Moved to: x={self.current_position.x}, y={self.current_position.y}")
        
        return {"success": True, "position": self.current_position.__dict__}
    
    def return_home(self) -> Dict:
        """
        返航命令
        
        Returns:
            {"success": bool, "position": DogPosition}
        """
        if self._ros_available:
            return self._return_home_ros()
        return self._return_home_mock()
    
    def _return_home_ros(self) -> Dict:
        """调用ROS服务返航"""
        try:
            import rospy
            from std_srvs.srv import Trigger, TriggerResponse
            
            # 调用返航服务
            rospy.wait_for_service('/return_home', timeout=5)
            response = rospy.ServiceProxy('/return_home', Trigger)()
            
            self.current_position = DogPosition()
            
            return {"success": response.success, "position": self.current_position.__dict__}
            
        except Exception as e:
            return {"success": False, "error": str(e), "position": self.current_position.__dict__}
    
    def _return_home_mock(self) -> Dict:
        """模拟返航"""
        print("[Dog Mock] Returning home...")
        time.sleep(0.1)
        
        self.current_position = DogPosition()
        print(f"[Dog Mock] Returned to home: {self.current_position.__dict__}")
        
        return {"success": True, "position": self.current_position.__dict__}
    
    def stop(self) -> Dict:
        """
        停止移动
        
        Returns:
            {"success": bool}
        """
        if self._ros_available:
            return self._stop_ros()
        return self._stop_mock()
    
    def _stop_ros(self) -> Dict:
        """调用ROS服务停止"""
        try:
            import rospy
            from geometry_msgs.msg import Twist
            
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            cmd_vel = Twist()
            pub.publish(cmd_vel)
            
            return {"success": True}
            
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def _stop_mock(self) -> Dict:
        """模拟停止"""
        print("[Dog Mock] Stopped")
        return {"success": True}
    
    def get_status(self) -> Dict:
        """
        获取当前状态
        
        Returns:
            {"state": str, "position": DogPosition}
        """
        return {
            "state": "idle",
            "position": self.current_position.__dict__
        }
    
    def is_ros_available(self) -> bool:
        """检查ROS是否可用"""
        return self._ros_available


# 测试代码
if __name__ == "__main__":
    adapter = MotionAdapter()
    
    print("\n=== Motion Adapter Test ===\n")
    print(f"ROS available: {adapter.is_ros_available()}")
    
    # 测试移动
    result = adapter.move(angle=45, duration=2)
    print(f"Move result: {result}")
    
    # 测试返航
    result = adapter.return_home()
    print(f"Return home result: {result}")
    
    # 测试停止
    result = adapter.stop()
    print(f"Stop result: {result}")
    
    # 获取状态
    status = adapter.get_status()
    print(f"Status: {status}")
