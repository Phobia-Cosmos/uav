#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SITL飞行测试脚本
起飞 → 圆形飞行 → 悬停 → 返航
"""
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative


class FlightTest:
    def __init__(self, connection_string='udp:127.0.0.1:14550', baud=57600):
        self.connection_string = connection_string
        self.baud = baud
        self.vehicle = None
        self.home_location = None
        
    def connect(self):
        """连接飞控"""
        print("=" * 60)
        print("   飞行测试程序")
        print("=" * 60)
        print(f"\n连接飞控: {self.connection_string}")
        
        self.vehicle = connect(self.connection_string, wait_ready=True, baud=self.baud, timeout=60)
        
        print(f"✓ 连接成功!")
        print(f"  模式: {self.vehicle.mode.name}")
        
        att = self.vehicle.attitude
        if att:
            print(f"  姿态: pitch={att.pitch:.2f}, roll={att.roll:.2f}, yaw={att.yaw:.2f}")
        
        loc = self.vehicle.location.global_relative_frame
        if loc:
            self.home_location = (loc.lat, loc.lon)
            print(f"  Home: ({loc.lat:.7f}, {loc.lon:.7f})")
        
        return True
    
    def arm_check(self):
        """解锁前检查"""
        print("\n[检查] 解锁前检查...")
        
        print(f"  飞控可解锁: {self.vehicle.is_armable}")
        print(f"  当前模式: {self.vehicle.mode.name}")
        print(f"  解锁状态: {self.vehicle.armed}")
        
        # 获取状态消息
        try:
            status = self.vehicle.system_status.state
            print(f"  系统状态: {status}")
        except:
            print("  系统状态: 未知")
        
        return self.vehicle.is_armable
    
    def arm_and_takeoff(self, altitude=5):
        """起飞到指定高度"""
        print(f"\n[Step 1] 起飞到 {altitude}米...")
        
        # 等待GPS位置估计和飞控可解锁
        print("  等待位置估计和飞控就绪...")
        for i in range(20):
            if self.vehicle.is_armable:
                print(f"  ✓ 飞控已就绪")
                break
            time.sleep(1)
        else:
            print("  ⚠ 等待超时，继续尝试...")
        
        # 切换到GUIDED模式
        print(f"  切换到GUIDED模式...")
        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(2)
        print(f"  当前模式: {self.vehicle.mode.name}")
        
        # 解锁
        print("  解锁中...")
        self.vehicle.armed = True
        
        # 切换到GUIDED模式
        print(f"  切换到GUIDED模式...")
        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(2)
        print(f"  当前模式: {self.vehicle.mode.name}")
        
        # 解锁
        print("  解锁中...")
        self.vehicle.armed = True
        time.sleep(3)
        
        if not self.vehicle.armed:
            print("  ✗ 解锁失败!")
            print("  可能原因: 未设置frame类型或参数错误")
            print("  尝试直接设置ARM...")
            # 发送MAVLink命令解锁
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,  # target system, component
                400,  # MAV_CMD_COMPONENT_ARM_DISARM
                0,  # confirmation
                1,  # arm (1=arm, 0=disarm)
                0, 0, 0, 0, 0, 0  # params
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            time.sleep(2)
            
            if self.vehicle.armed:
                print("  ✓ 解锁成功!")
            else:
                # 显示更多诊断信息
                print("  ✗ 仍然无法解锁")
                print(f"  系统状态: {self.vehicle.system_status.state}")
                print(f"  飞控可解锁: {self.vehicle.is_armable}")
                
                # 尝试获取更多状态信息
                try:
                    # 获取所有通道的RC值
                    rc_channels = self.vehicle.channels
                    print(f"  RC通道: {rc_channels}")
                except:
                    pass
                
                return False
        else:
            print("  ✓ 已解锁")
        
        # 起飞
        print("  起飞中...")
        self.vehicle.simple_takeoff(altitude)
        
        # 等待到达目标高度
        for i in range(30):
            if not self.vehicle.armed:
                print("  ✗ 已锁定")
                return False
                
            current_alt = self.vehicle.location.global_relative_frame.alt
            print(f"  当前高度: {current_alt:.1f}m")
            
            if current_alt >= altitude * 0.95:
                print(f"  ✓ 到达目标高度 {altitude}米")
                return True
                
            time.sleep(1)
        
        print("  ⚠ 等待超时，仍在上升中...")
        return True
    
    def fly_circle(self, radius=10, altitude=10, duration=20):
        """绕圈飞行"""
        print(f"\n[Step 2] 绕圈飞行 (半径={radius}米, 高度={altitude}米)")
        
        # 切换到GUIDED模式（如果还没切换）
        if self.vehicle.mode.name != "GUIDED":
            print("  切换到GUIDED模式...")
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(2)
        
        current_loc = self.vehicle.location.global_relative_frame
        if not current_loc:
            print("  ✗ 无法获取位置")
            return False
        
        center_lat = current_loc.lat
        center_lon = current_loc.lon
        
        # 先上升到指定高度
        if current_loc.alt < altitude - 2:
            print(f"  上升到 {altitude}米...")
            target = LocationGlobalRelative(center_lat, center_lon, altitude)
            self.vehicle.simple_goto(target)
            time.sleep(5)
        
        # 计算移动速度
        speed_mps = 5  # 5 m/s
        angle_step = 10  # 每步10度
        
        print("  开始绕圈...")
        start_time = time.time()
        angle = 0
        
        while time.time() - start_time < duration:
            # 计算圆周位置
            lat_offset = (radius * math.cos(math.radians(angle))) / 111000
            lon_offset = (radius * math.sin(math.radians(angle))) / (111000 * math.cos(math.radians(center_lat)))
            
            target = LocationGlobalRelative(
                center_lat + lat_offset,
                center_lon + lon_offset,
                altitude
            )
            
            # 发送导航指令
            self.vehicle.simple_goto(target)
            
            # 显示进度
            elapsed = time.time() - start_time
            pos = self.vehicle.location.global_relative_frame
            if pos:
                print(f"  [{elapsed:.1f}s] 位置: ({pos.lat:.6f}, {pos.lon:.6f}), 高度: {pos.alt:.1f}m")
            
            # 更新角度
            angle += angle_step
            if angle >= 360:
                angle = 0
            
            time.sleep(1)
        
        print("  ✓ 绕圈完成")
        return True
    
    def hover_and_wait(self, duration=5):
        """悬停等待"""
        print(f"\n[Step 3] 悬停等待 {duration}秒...")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            remaining = int(duration - (time.time() - start_time))
            pos = self.vehicle.location.global_relative_frame
            if pos:
                print(f"  剩余 {remaining}秒 | 位置: ({pos.lat:.6f}, {pos.lon:.6f}) | 高度: {pos.alt:.1f}m")
            time.sleep(1)
        
        print("  ✓ 悬停完成")
        return True
    
    def return_to_home(self):
        """返航"""
        print(f"\n[Step 4] 返回Home...")
        
        if not self.home_location:
            print("  ✗ 没有Home位置")
            return False
        
        lat, lon = self.home_location
        print(f"  Home: ({lat:.7f}, {lon:.7f})")
        
        # 飞向Home（保持当前高度）
        current_alt = self.vehicle.location.global_relative_frame.alt
        target = LocationGlobalRelative(lat, lon, max(current_alt, 10))
        
        print("  飞向Home...")
        self.vehicle.simple_goto(target)
        
        # 等待到达
        for i in range(15):
            pos = self.vehicle.location.global_relative_frame
            if pos:
                # 计算到Home的距离
                dist = math.sqrt(
                    ((pos.lat - lat) * 111000) ** 2 +
                    ((pos.lon - lon) * 111000 * math.cos(math.radians(lat))) ** 2
                )
                print(f"  距离Home: {dist:.1f}m")
                if dist < 5:
                    break
            time.sleep(1)
        
        print("  ✓ 已返回Home区域")
        return True
    
    def land(self):
        """降落"""
        print(f"\n[Step 5] 降落...")
        
        self.vehicle.mode = VehicleMode("LAND")
        
        while True:
            pos = self.vehicle.location.global_relative_frame
            if pos:
                print(f"  高度: {pos.alt:.1f}m")
                if pos.alt < 1:
                    print("  ✓ 降落完成")
                    break
            time.sleep(1)
        
        return True
    
    def run_full_test(self, radius=15, altitude=10):
        """完整测试"""
        try:
            if not self.connect():
                return False
            
            if not self.arm_and_takeoff(altitude):
                return False
            
            if not self.fly_circle(radius=radius, altitude=altitude, duration=20):
                return False
            
            if not self.hover_and_wait(duration=5):
                return False
            
            if not self.return_to_home():
                return False
            
            if not self.land():
                return False
            
            print("\n" + "=" * 60)
            print("   测试完成!")
            print("=" * 60)
            return True
            
        except Exception as e:
            print(f"\n错误: {e}")
            import traceback
            traceback.print_exc()
            return False
        
        finally:
            if self.vehicle:
                try:
                    self.vehicle.armed = False
                    self.vehicle.close()
                except:
                    pass
                print("\n连接已关闭")
    
    def quick_circle_test(self, radius=15, duration=15):
        """快速绕圈测试"""
        try:
            if not self.connect():
                return False
            
            if not self.arm_and_takeoff(altitude=10):
                return False
            
            if not self.fly_circle(radius=radius, altitude=10, duration=duration):
                return False
            
            if not self.land():
                return False
            
            print("\n快速测试完成!")
            return True
            
        except Exception as e:
            print(f"错误: {e}")
            return False
        
        finally:
            if self.vehicle:
                try:
                    self.vehicle.close()
                except:
                    pass


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='飞行测试脚本')
    parser.add_argument('--connection', default='udp:127.0.0.1:14550',
                        help='飞控连接串')
    parser.add_argument('--baud', type=int, default=57600, help='波特率')
    parser.add_argument('--mode', default='quick',
                        choices=['full', 'quick', 'circle', 'hover', 'arm_test'],
                        help='测试模式')
    parser.add_argument('--radius', type=float, default=15, help='绕圈半径（米）')
    parser.add_argument('--altitude', type=float, default=10, help='飞行高度（米）')
    parser.add_argument('--time', type=float, default=15, help='持续时间（秒）')
    
    args = parser.parse_args()
    
    test = FlightTest(args.connection, args.baud)
    
    if args.mode == 'full':
        test.run_full_test(radius=args.radius, altitude=args.altitude)
    elif args.mode == 'quick':
        test.quick_circle_test(radius=args.radius, duration=args.time)
    elif args.mode == 'circle':
        test.connect()
        test.arm_and_takeoff(altitude=args.altitude)
        test.fly_circle(radius=args.radius, altitude=args.altitude, duration=args.time)
        test.land()
    elif args.mode == 'hover':
        test.connect()
        test.arm_and_takeoff(altitude=args.altitude)
        test.hover_and_wait(duration=args.time)
    elif args.mode == 'arm_test':
        test.connect()
        test.arm_check()


if __name__ == '__main__':
    main()
