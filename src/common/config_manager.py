#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Configuration Manager

智能配置管理器，根据WiFi自动选择对应的IP配置。
支持多种启动参数方式配置。
"""

import json
import os
import subprocess
import argparse
from pathlib import Path
from typing import Dict, Optional, Any


class ConfigManager:
    """配置管理器"""

    def __init__(self, config_file: str = None):
        """
        初始化配置管理器
        
        Args:
            config_file: 配置文件路径
        """
        if config_file is None:
            # TODO：可以这样命名吗？
            config_file = Path(__file__).parent.parent / "config" / "network_config.json"
        
        self.config_file = Path(config_file)
        self.config = self._load_config()
        self.override: Dict[str, Any] = {}
        self.current_wifi: Optional[str] = None
        self.device_type: Optional[str] = None

    def _load_config(self) -> Dict:
        """加载配置文件"""
        if not self.config_file.exists():
            raise FileNotFoundError(f"配置文件不存在: {self.config_file}")
        
        with open(self.config_file, 'r', encoding='utf-8') as f:
            return json.load(f)

    def detect_wifi(self) -> Optional[str]:
        """
        检测当前连接的WiFi名称
        
        Returns:
            WiFi名称或None
        """
        try:
            # TODO：nmcli具体是如何使用的？可以使用哪些方式查询wifi？
            result = subprocess.run(
                ["nmcli", "-t", "-f", "ACTIVE,SSID", "device", "wifi"],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            for line in result.stdout.strip().split('\n'):
                if line.startswith('yes:'):
                    wifi_name = line.split(':', 1)[1]
                    self.current_wifi = wifi_name
                    return wifi_name
                    
        except Exception as e:
            pass
        
        return None

    def get_wifi_profile(self, wifi_name: str = None) -> Dict:
        """
        获取指定WiFi的配置
        
        Args:
            wifi_name: WiFi名称，为None时自动检测
            
        Returns:
            WiFi配置
        """
        if wifi_name is None:
            wifi_name = self.current_wifi or self.detect_wifi()
        
        wifi_profiles = self.config.get("wifi_profiles", {})
        
        # 尝试精确匹配
        if wifi_name in wifi_profiles:
            return wifi_profiles[wifi_name]
        
        # 使用默认配置
        default_wifi = self.config.get("default_wifi", "2楼")
        if default_wifi in wifi_profiles:
            return wifi_profiles[default_wifi]
        
        # 返回空配置
        return {}

    def get_device_config(self, device_type: str) -> Dict:
        """
        获取指定设备的配置
        
        Args:
            device_type: 设备类型 (pc/drone/dog)
            
        Returns:
            设备配置
        """
        # 1. 检查override
        if self.override:
            return self.override.get(device_type, {})
        
        # 2. 根据WiFi获取配置
        profile = self.get_wifi_profile()
        return profile.get(device_type, {})

    def get_ip(self, device_type: str) -> str:
        """获取设备IP地址"""
        device_config = self.get_device_config(device_type)
        return device_config.get("ip", "")

    def get_port(self, port_name: str, device_type: str = None) -> int:
        """
        获取端口配置
        
        Args:
            port_name: 端口名称
            device_type: 设备类型（可选）
            
        Returns:
            端口号
        """
        # 端口不随WiFi变化，从默认配置获取
        ports = self.config.get("ports", {})
        
        # 尝试从设备配置获取
        if device_type:
            device_config = self.get_device_config(device_type)
            if "ports" in device_config:
                ports = device_config["ports"]
        
        return ports.get(port_name, 0)

    def get_heartbeat_config(self) -> Dict:
        """获取心跳配置"""
        return self.config.get("heartbeat", {"interval": 10, "timeout": 30})

    def get_flight_config(self) -> Dict:
        """获取飞行配置"""
        return self.config.get("flight", {"default_altitude": 3.0})

    def get_control_config(self) -> Dict:
        """获取控制配置"""
        return self.config.get("control", {"move_interval": 10})

    def set_override(self, device_type: str, config: Dict):
        """
        设置覆盖配置
        
        Args:
            device_type: 设备类型
            config: 配置字典
        """
        self.override[device_type] = config

    def clear_override(self):
        """清除覆盖配置"""
        self.override = {}

    def get_connection_info(self, device_type: str) -> tuple:
        """
        获取设备连接信息
        
        Returns:
            (ip, port) 元组
        """
        device_config = self.get_device_config(device_type)
        ip = device_config.get("ip", "")
        
        # 根据设备类型获取对应端口
        if device_type == "pc":
            port = self.get_port("from_drone")
        elif device_type == "drone":
            port = self.get_port("from_pc")
        elif device_type == "dog":
            port = self.get_port("to_dog")
        else:
            port = 0
            
        return ip, port

    def to_dict(self) -> Dict:
        """导出当前配置"""
        return {
            "current_wifi": self.current_wifi,
            "device_type": self.device_type,
            "pc": self.get_device_config("pc"),
            "drone": self.get_device_config("drone"),
            "dog": self.get_device_config("dog"),
            "heartbeat": self.get_heartbeat_config()
        }


def parse_args() -> argparse.Namespace:
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="Ground-Air Cooperation System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python main.py --auto                    # 自动检测WiFi
  python main.py --wifi "2楼"              # 指定WiFi网络
  python main.py --device drone            # 指定设备类型
  python main.py --pc-ip 192.168.1.100    # 手动指定PC IP
        """
    )
    
    parser.add_argument(
        "--auto",
        action="store_true",
        help="自动检测当前WiFi并选择配置"
    )
    
    parser.add_argument(
        "--wifi",
        type=str,
        help="手动指定WiFi网络名称"
    )
    
    parser.add_argument(
        "--device",
        type=str,
        choices=["pc", "drone", "dog"],
        help="手动指定设备类型"
    )
    
    parser.add_argument(
        "--pc-ip",
        type=str,
        help="手动指定PC IP地址"
    )
    
    parser.add_argument(
        "--drone-ip",
        type=str,
        help="手动指定无人机IP"
    )
    
    parser.add_argument(
        "--dog-ip",
        type=str,
        help="手动指定机器狗IP"
    )
    
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="指定配置文件路径"
    )
    
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="日志级别"
    )
    
    return parser.parse_args()


def apply_args(config: ConfigManager, args: argparse.Namespace) -> ConfigManager:
    """
    应用命令行参数到配置
    
    Args:
        config: 配置管理器
        args: 命令行参数
        
    Returns:
        更新后的配置管理器
    """
    # 设置设备类型
    if args.device:
        config.device_type = args.device
    
    # 应用手动指定IP
    override = {}
    if args.pc_ip:
        override["pc"] = {"ip": args.pc_ip}
    if args.drone_ip:
        override["drone"] = {"ip": args.drone_ip}
    if args.dog_ip:
        override["dog"] = {"ip": args.dog_ip}
    
    if override:
        for device_type, device_config in override.items():
            config.set_override(device_type, device_config)
    
    return config


def get_config(config_file: str = None) -> ConfigManager:
    """
    获取配置管理器实例
    
    Args:
        config_file: 配置文件路径
        
    Returns:
        配置管理器实例
    """
    return ConfigManager(config_file)


# 测试代码
if __name__ == "__main__":
    import sys
    
    args = parse_args()
    config = get_config(args.config)
    
    # 应用参数
    config = apply_args(config, args)
    
    # 检测WiFi
    if args.auto or args.wifi:
        wifi_name = args.wifi or config.detect_wifi()
        print(f"当前WiFi: {wifi_name}")
    
    # 显示配置
    print("\n当前配置:")
    print(f"  PC:     {config.get_ip('pc')}:{config.get_port('from_drone', 'pc')}")
    print(f"  Drone:  {config.get_ip('drone')}:{config.get_port('from_pc', 'drone')}")
    print(f"  Dog:    {config.get_ip('dog')}:{config.get_port('to_dog', 'dog')}")
    print(f"  心跳:   {config.get_heartbeat_config()}")
