#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Flight Controller for Drone

无人机飞行控制模块，基于dronekit。
"""

import time
from typing import Optional, Dict
from dataclasses import dataclass
from enum import Enum


class FlightState(Enum):
    """飞行状态"""
    DISARMED = "disarmed"
    ARMED = "armed"
    TAKING_OFF = "taking_off"
    HOVERING = "hovering"
    MOVING = "moving"
    RETURNING = "returning"
    LANDING = "landing"
    LANDED = "landed"
    ERROR = "error"


@dataclass
class FlightStatus:
    """飞行状态数据"""
    state: str = FlightState.DISARMED.value
    altitude: float = 0.0
    battery: int = 100
    latitude: float = 0.0
    longitude: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0


class FlightController:
    """飞行控制器"""
    
    def __init__(self, connection_string: str = "/dev/ttyACM0", baud: int = 921600):
        """
        初始化飞行控制器
        
        Args:
            connection_string: 飞控连接字符串
            baud: 波特率
        """
        self.connection_string = connection_string
        self.baud = baud
        
        self.vehicle = None
        self.state = FlightState.DISARMED
        self.status = FlightStatus()
        
        self._logger = None
    
    def set_logger(self, logger):
        """设置日志器"""
        self._logger = logger
    
    def connect(self) -> bool:
        """连接飞控"""
        try:
            from dronekit import connect, VehicleMode
            
            self._logger.info(f"Connecting to vehicle: {self.connection_string}")
            self.vehicle = connect(
                self.connection_string,
                wait_ready=True,
                baud=self.baud
            )
            
            self._logger.info("Vehicle connected successfully")
            self._logger.info(f"Mode: {self.vehicle.mode.name}")
            self._logger.info(f"Armed: {self.vehicle.armed}")
            
            return True
            
        except Exception as e:
            self._logger.error(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """断开飞控连接"""
        if self.vehicle:
            try:
                self.vehicle.close()
            except Exception as e:
                self._logger.error(f"Error closing vehicle: {e}")
            self.vehicle = None
    
    def update_status(self):
        """更新状态"""
        if not self.vehicle:
            return
        
        try:
            loc = self.vehicle.location.global_relative_frame
            battery = self.vehicle.battery
            
            self.status.altitude = loc.alt if loc and loc.alt else 0
            self.status.latitude = loc.lat if loc and loc.lat else 0
            self.status.longitude = loc.lon if loc and loc.lon else 0
            
            if battery:
                self.status.battery = getattr(battery, 'level', 100)
            
            att = self.vehicle.attitude
            if att:
                self.status.pitch = att.pitch
                self.status.roll = att.roll
                self.status.yaw = att.yaw
            
            self.status.state = self.state.value
            
        except Exception as e:
            self._logger.error(f"Error updating status: {e}")
    
    def is_armable(self) -> bool:
        """检查是否可解锁"""
        if not self.vehicle:
            return False
        return self.vehicle.is_armable
    
    def arm(self) -> bool:
        """解锁"""
        if not self.vehicle:
            return False
        
        try:
            from dronekit import VehicleMode
            
            self.state = FlightState.ARMED
            self._logger.info("Arming motors...")
            
            self.vehicle.armed = True
            
            # 等待解锁
            for _ in range(30):
                if self.vehicle.armed:
                    self._logger.info("Motors armed")
                    return True
                time.sleep(0.1)
            
            self._logger.warning("Arming timeout")
            return False
            
        except Exception as e:
            self._logger.error(f"Arming failed: {e}")
            return False
    
    def disarm(self) -> bool:
        """上锁"""
        if not self.vehicle:
            return False
        
        try:
            self._logger.info("Disarming motors...")
            self.vehicle.armed = False
            
            for _ in range(30):
                if not self.vehicle.armed:
                    self.state = FlightState.DISARMED
                    self._logger.info("Motors disarmed")
                    return True
                time.sleep(0.1)
            
            self._logger.warning("Disarming timeout")
            return False
            
        except Exception as e:
            self._logger.error(f"Disarming failed: {e}")
            return False
    
    def takeoff(self, altitude: float) -> bool:
        """
        起飞
        
        Args:
            altitude: 目标高度（米）
            
        Returns:
            是否成功
        """
        if not self.vehicle:
            return False
        
        try:
            from dronekit import VehicleMode
            
            self.state = FlightState.TAKING_OFF
            self._logger.info(f"Taking off to {altitude}m...")
            
            # 切换到GUIDED模式
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)
            
            # 解锁
            if not self.vehicle.armed:
                if not self.arm():
                    return False
            
            # 起飞
            self.vehicle.simple_takeoff(altitude)
            
            # 等待到达目标高度
            for _ in range(60):
                self.update_status()
                if self.status.altitude >= altitude * 0.95:
                    self.state = FlightState.HOVERING
                    self._logger.info(f"Reached target altitude: {self.status.altitude:.1f}m")
                    return True
                time.sleep(0.5)
            
            self._logger.warning("Takeoff timeout, still climbing")
            self.state = FlightState.HOVERING
            return True
            
        except Exception as e:
            self._logger.error(f"Takeoff failed: {e}")
            self.state = FlightState.ERROR
            return False
    
    def land(self) -> bool:
        """降落"""
        if not self.vehicle:
            return False
        
        try:
            from dronekit import VehicleMode
            
            self.state = FlightState.LANDING
            self._logger.info("Landing...")
            
            self.vehicle.mode = VehicleMode("LAND")
            
            # 等待落地
            for _ in range(120):
                self.update_status()
                if self.status.altitude < 0.5:
                    self.state = FlightState.LANDED
                    self._logger.info("Landed")
                    return True
                time.sleep(0.5)
            
            self._logger.warning("Landing timeout")
            return False
            
        except Exception as e:
            self._logger.error(f"Landing failed: {e}")
            return False
    
    def return_to_launch(self, return_altitude: float = 5.0) -> bool:
        """
        返航
        
        Args:
            return_altitude: 返航高度
        """
        if not self.vehicle:
            return False
        
        try:
            from dronekit import VehicleMode
            
            self.state = FlightState.RETURNING
            self._logger.info("Returning to launch...")
            
            # 设置返航高度
            self.vehicle.parameters['RTL_ALT'] = return_altitude * 100  # cm
            
            # 触发返航
            self.vehicle.mode = VehicleMode("RTL")
            
            # 等待返航完成
            home_loc = self.vehicle.home_location
            if home_loc:
                home_lat = home_loc.lat
                home_lon = home_loc.lon
                
                for _ in range(180):
                    self.update_status()
                    
                    # 检查是否接近home
                    if self.status.latitude and self.status.longitude:
                        dist = self._calculate_distance(
                            self.status.latitude, self.status.longitude,
                            home_lat, home_lon
                        )
                        if dist < 10 and self.status.altitude < return_altitude + 2:
                            self._logger.info("Returned to launch")
                            return True
                    
                    # 检查是否落地
                    if self.status.altitude < 0.5:
                        self._logger.info("Landed at home")
                        return True
                    
                    time.sleep(1)
            
            self._logger.warning("Return timeout")
            return False
            
        except Exception as e:
            self._logger.error(f"Return failed: {e}")
            return False
    
    def hover(self, duration: float) -> bool:
        """
        悬停指定时间
        
        Args:
            duration: 悬停时间（秒）
        """
        if not self.vehicle:
            return False
        
        self.state = FlightState.HOVERING
        start = time.time()
        
        while time.time() - start < duration:
            self.update_status()
            if self.state != FlightState.HOVERING:
                return False
            time.sleep(0.5)
        
        return True
    
    def set_mode(self, mode: str) -> bool:
        """
        设置飞行模式
        
        Args:
            mode: 模式名称
        """
        if not self.vehicle:
            return False
        
        try:
            from dronekit import VehicleMode
            self.vehicle.mode = VehicleMode(mode)
            self._logger.info(f"Mode set to {mode}")
            return True
        except Exception as e:
            self._logger.error(f"Failed to set mode: {e}")
            return False
    
    def _calculate_distance(self, lat1, lon1, lat2, lon2) -> float:
        """计算两点间距离（米）"""
        import math
        R = 6371000  # 地球半径（米）
        
        lat1_rad = lat1 * 3.14159 / 180
        lat2_rad = lat2 * 3.14159 / 180
        delta_lat = (lat2 - lat1) * 3.14159 / 180
        delta_lon = (lon2 - lon1) * 3.14159 / 180
        
        a = (delta_lat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * (delta_lon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c


if __name__ == "__main__":
    # 测试代码
    import sys
    sys.path.insert(0, str(__file__).rsplit('/', 3)[0])
    
    from common import get_logger
    
    logger = get_logger("flight_test", level="DEBUG")
    
    controller = FlightController()
    controller.set_logger(logger)
    
    print("\n=== Flight Controller Test ===\n")
    
    if controller.connect():
        print("Connected to vehicle")
        
        print(f"Armable: {controller.is_armable()}")
        print(f"Status: {controller.status}")
        
        # 测试起飞
        print("\nTaking off...")
        if controller.takeoff(3.0):
            print("Takeoff successful")
            
            print("\nHovering for 5 seconds...")
            controller.hover(5)
            
            print("\nLanding...")
            controller.land()
        
        controller.disarm()
        controller.disconnect()
    
    print("\nTest completed")
