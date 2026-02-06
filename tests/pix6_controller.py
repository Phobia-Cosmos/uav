#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pix6飞控无RC控制模块
用于禁用RC检测和保护，方便纯代码控制飞控

使用方法:
    from pix6_controller import Pix6Controller
    
    pix6 = Pix6Controller('/dev/ttyACM0', 921600)
    pix6.connect()
    pix6.disable_rc_requirement()  # 禁用RC检测
    ...
    pix6.disarm()  # 解锁
    pix6.close()   # 关闭连接
"""

import time
from dronekit import connect, VehicleMode


class Pix6Controller:
    """Pix6飞控无RC控制类"""
    
    def __init__(self, connection_string='/dev/ttyACM0', baudrate=921600):
        """
        初始化Pix6Controller
        
        Args:
            connection_string: 连接字符串，如 '/dev/ttyACM0' 或 'udp:127.0.0.1:14550'
            baudrate: 波特率，默认921600
        """
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.vehicle = None
        self.connected = False
    
    def connect(self):
        """连接飞控"""
        print(f"Connecting to vehicle on: {self.connection_string}")
        self.vehicle = connect(
            self.connection_string, 
            wait_ready=True, 
            baud=self.baudrate,
            timeout=30
        )
        self.connected = True
        print(f"  飞控已连接")
        print(f"  固件版本: {self.vehicle.version}")
        return self.vehicle
    
    def disable_rc_requirement(self):
        """
        禁用RC检测和失控保护
        必须在解锁前调用
        """
        if not self.connected:
            raise RuntimeError("请先调用 connect() 连接飞控")
        
        print("禁用RC和失控保护...")
        
        params_to_set = [
            ('BRD_RC_TYPES', 0),      # 禁用RC类型检测
            ('FS_THR_ENABLE', 0),     # 禁用油门失控保护
        ]
        
        success_count = 0
        for param_name, value in params_to_set:
            try:
                self.vehicle.parameters[param_name] = value
                print(f"  {param_name} = {value} ✓")
                time.sleep(0.5)
                success_count += 1
            except Exception as e:
                print(f"  {param_name} 设置失败: {e}")
        
        if success_count == 0:
            print("  警告: 所有参数设置失败，可能需要手动设置")
        
        return success_count
    
    def set_mode(self, mode_name):
        """
        设置飞控模式
        
        Args:
            mode_name: 模式名称，如 'STABILIZE', 'ALT_HOLD', 'GUIDED' 等
        """
        if not self.connected:
            raise RuntimeError("请先调用 connect() 连接飞控")
        
        print(f"切换到 {mode_name} 模式...")
        self.vehicle.mode = VehicleMode(mode_name)
        
        # 等待模式切换完成
        timeout = 10
        start = time.time()
        while self.vehicle.mode.name != mode_name:
            if time.time() - start > timeout:
                raise RuntimeError(f"模式切换超时")
            time.sleep(0.2)
        
        print(f"  模式切换成功: {self.vehicle.mode.name}")
    
    def arm(self, check_armable=False):
        """
        解锁飞控
        
        Args:
            check_armable: 是否等待飞控可解锁状态，默认False（无RC时用）
        """
        if not self.connected:
            raise RuntimeError("请先调用 connect() 连接飞控")
        
        print("解锁飞控...")
        
        if check_armable:
            # 等待飞控可解锁
            timeout = 30
            start = time.time()
            while not self.vehicle.is_armable:
                if time.time() - start > timeout:
                    raise RuntimeError("飞控不可解锁")
                time.sleep(0.5)
            print("  飞控已就绪")
        
        self.vehicle.armed = True
        
        # 等待解锁完成
        timeout = 10
        start = time.time()
        while not self.vehicle.armed:
            if time.time() - start > timeout:
                raise RuntimeError("解锁失败")
            time.sleep(0.2)
        
        print("  解锁成功!")
    
    def disarm(self):
        """锁定飞控"""
        if not self.connected:
            return
        
        print("锁定飞控...")
        
        if not self.vehicle.armed:
            print("  飞控已锁定")
            return
        
        # 清除所有override
        try:
            self.vehicle.channels.overrides = {}
            print("  已清除RC override")
        except:
            pass
        
        # 等待一下让飞控处理
        time.sleep(1)
        
        # 尝试使用force disarm
        try:
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,           # target system, component
                400,            # MAV_CMD_COMPONENT_ARM_DISARM
                0,              # confirmation
                0,              # param1: 0=disarm
                21196,          # param2: force=21196
                0, 0, 0, 0, 0   # params
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            print("  已发送强制解锁命令")
            time.sleep(1)
        except Exception as e:
            print(f"  强制解锁失败: {e}")
        
        # 尝试普通解锁
        try:
            self.vehicle.armed = False
        except:
            pass
        
        # 等待解锁完成
        timeout = 5
        start = time.time()
        while self.vehicle.armed:
            if time.time() - start > timeout:
                print("  警告: 飞控未响应，解锁状态仍为True")
                break
            time.sleep(0.2)
        
        print("  完成")
    
    def set_throttle(self, pwm_value):
        """
        设置油门override
        
        Args:
            pwm_value: PWM值，范围通常是 1000-2000
        """
        if not self.connected:
            raise RuntimeError("请先调用 connect() 连接飞控")
        
        self.vehicle.channels.overrides = {'3': pwm_value}
        print(f"  油门设置为: {pwm_value}")
    
    def clear_throttle(self):
        """清除油门override"""
        if not self.connected:
            return
        
        try:
            self.vehicle.channels.overrides = {}
            print("  油门override已清除")
        except Exception as e:
            print(f"  清除失败: {e}")
    
    def get_status(self):
        """获取飞控状态"""
        if not self.connected:
            return "未连接"
        
        status = {
            'mode': self.vehicle.mode.name if self.vehicle.mode else '未知',
            'armed': self.vehicle.armed,
            'system_status': self.vehicle.system_status.state if self.vehicle.system_status else '未知',
            'location': self.vehicle.location.global_relative_frame if self.vehicle.location else None,
        }
        
        return status
    
    def close(self):
        """关闭连接"""
        if self.connected and self.vehicle:
            print("关闭飞控连接...")
            try:
                self.vehicle.close()
            except:
                pass
            self.connected = False
            print("  完成")
    
    def __enter__(self):
        """支持with语句"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """支持with语句自动清理"""
        self.disarm()
        self.clear_throttle()
        self.close()
        return False


# 便捷函数
def connect_pix6(connection_string='/dev/ttyACM0', baudrate=921600):
    """
    快速连接并禁用RC检测
    
    Returns:
        Pix6Controller实例
    """
    pix6 = Pix6Controller(connection_string, baudrate)
    pix6.connect()
    pix6.disable_rc_requirement()
    return pix6


if __name__ == '__main__':
    # 测试代码
    print("=" * 50)
    print("Pix6 Controller 测试")
    print("=" * 50)
    
    try:
        with Pix6Controller('/dev/ttyACM0', 921600) as pix6:
            print("\n获取飞控状态...")
            status = pix6.get_status()
            for key, value in status.items():
                print(f"  {key}: {value}")
            
            print("\n禁用RC检测...")
            pix6.disable_rc_requirement()
            
            print("\n测试完成!")
            
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
