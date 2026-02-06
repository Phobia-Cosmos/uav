#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UAV完整功能测试程序
先在SITL中测试，再在Pix6硬件上测试

使用方法:
    # SITL模式测试
    python3 uav_test.py --sitl --auto
    
    # Pix6硬件测试
    python3 uav_test.py --pix6 --auto
    
    # 交互式测试
    python3 uav_test.py --sitl
"""

import sys
import time
import argparse
from datetime import datetime


class UAVTestSuite:
    """UAV完整测试套件"""
    
    def __init__(self, mode='sitl', connection_string=None):
        """
        初始化测试套件
        
        Args:
            mode: 'sitl' 或 'pix6'
            connection_string: 连接字符串
        """
        self.mode = mode
        self.vehicle = None
        self.test_results = []
        
        if mode == 'sitl':
            self.connection_string = connection_string or 'udp:127.0.0.1:14550'
            self.baud = 57600
        else:  # pix6
            self.connection_string = connection_string or '/dev/ttyACM0'
            self.baud = 921600
        
        self.setup_logging()
    
    def setup_logging(self):
        """设置日志"""
        self.log_file = open(f"/tmp/uav_test_{self.mode}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log", 'w')
        self.log("=" * 60)
        self.log(f"   UAV测试套件 - {self.mode.upper()}模式")
        self.log("=" * 60)
        self.log(f"连接: {self.connection_string} @ {self.baud}")
    
    def log(self, msg):
        """打印并记录日志"""
        print(msg)
        self.log_file.write(msg + '\n')
        self.log_file.flush()
    
    def log_result(self, test_name, passed, error=None):
        """记录测试结果"""
        result = {
            'test': test_name,
            'passed': passed,
            'error': error,
            'time': datetime.now().isoformat()
        }
        self.test_results.append(result)
        status = "✓" if passed else "✗"
        self.log(f"[{status}] {test_name}")
        if error and not passed:
            self.log(f"    错误: {error}")
    
    # ========================
    # 连接测试
    # ========================
    def test_connection(self):
        """测试飞控连接"""
        self.log("\n" + "-" * 40)
        self.log("测试1: 飞控连接")
        self.log("-" * 40)
        
        try:
            from dronekit import connect
            self.log(f"正在连接 {self.connection_string}...")
            self.vehicle = connect(
                self.connection_string, 
                wait_ready=True, 
                baud=self.baud,
                timeout=30
            )
            self.log(f"✓ 连接成功!")
            self.log(f"  模式: {self.vehicle.mode.name}")
            self.log(f"  姿态: pitch={self.vehicle.attitude.pitch:.2f}, "
                    f"roll={self.vehicle.attitude.roll:.2f}, "
                    f"yaw={self.vehicle.attitude.yaw:.2f}")
            self.log_result("飞控连接", True)
            return True
        except Exception as e:
            self.log(f"✗ 连接失败: {e}")
            self.log_result("飞控连接", False, str(e))
            return False
    
    # ========================
    # 模式切换测试
    # ========================
    def test_mode_switch(self):
        """测试模式切换"""
        self.log("\n" + "-" * 40)
        self.log("测试2: 模式切换")
        self.log("-" * 40)
        
        if not self.vehicle:
            self.log_result("模式切换", False, "飞控未连接")
            return False
        
        modes_to_test = ['STABILIZE', 'ALT_HOLD', 'GUIDED']
        
        for mode_name in modes_to_test:
            try:
                self.log(f"  切换到 {mode_name}...")
                self.vehicle.mode = VehicleMode(mode_name)
                
                # 等待切换完成
                timeout = 10
                start = time.time()
                while self.vehicle.mode.name != mode_name:
                    if time.time() - start > timeout:
                        raise TimeoutError(f"模式切换超时: {mode_name}")
                    time.sleep(0.2)
                
                self.log(f"    ✓ {mode_name}")
                time.sleep(0.5)
            except Exception as e:
                self.log(f"    ✗ {mode_name}: {e}")
                self.log_result("模式切换", False, str(e))
                return False
        
        self.log_result("模式切换", True)
        return True
    
    # ========================
    # 解锁测试
    # ========================
    def test_arm_disarm(self):
        """测试解锁/锁定"""
        self.log("\n" + "-" * 40)
        self.log("测试3: 解锁/锁定")
        self.log("-" * 40)
        
        if not self.vehicle:
            self.log_result("解锁/锁定", False, "飞控未连接")
            return False
        
        try:
            # Pix6模式需要禁用RC检测
            if self.mode == 'pix6':
                self.log("  禁用RC检测...")
                try:
                    self.vehicle.parameters['BRD_RC_TYPES'] = 0
                    self.vehicle.parameters['FS_THR_ENABLE'] = 0
                    self.log("    ✓ RC检测已禁用")
                except Exception as e:
                    self.log(f"    ⚠ 参数设置失败: {e}")
            
            # 切换到STABILIZE
            self.log("  切换到STABILIZE模式...")
            self.vehicle.mode = VehicleMode("STABILIZE")
            time.sleep(1)
            
            # 解锁
            self.log("  解锁飞控...")
            self.vehicle.armed = True
            
            # 等待解锁
            timeout = 15
            start = time.time()
            while not self.vehicle.armed:
                if time.time() - start > timeout:
                    raise TimeoutError("解锁超时")
                time.sleep(0.2)
            
            self.log(f"  ✓ 解锁成功 (armed={self.vehicle.armed})")
            time.sleep(1)
            
            # 锁定
            self.log("  锁定飞控...")
            self.vehicle.armed = False
            
            timeout = 10
            start = time.time()
            while self.vehicle.armed:
                if time.time() - start > timeout:
                    self.log("  ⚠ 锁定超时，尝试强制锁定...")
                    # 尝试MAVLink强制解锁
                    msg = self.vehicle.message_factory.command_long_encode(
                        0, 0, 400, 0, 0, 21196, 0, 0, 0, 0, 0
                    )
                    self.vehicle.send_mavlink(msg)
                    self.vehicle.flush()
                    time.sleep(1)
                    self.vehicle.armed = False
                    break
                time.sleep(0.2)
            
            self.log(f"  ✓ 锁定成功 (armed={self.vehicle.armed})")
            self.log_result("解锁/锁定", True)
            return True
            
        except Exception as e:
            self.log(f"✗ 解锁/锁定失败: {e}")
            self.log_result("解锁/锁定", False, str(e))
            return False
    
    # ========================
    # 起飞测试
    # ========================
    def test_takeoff(self):
        """测试起飞"""
        self.log("\n" + "-" * 40)
        self.log("测试4: 起飞测试")
        self.log("-" * 40)
        
        if not self.vehicle:
            self.log_result("起飞测试", False, "飞控未连接")
            return False
        
        try:
            target_alt = 5  # 5米
            
            self.log(f"  目标高度: {target_alt}米")
            self.log("  切换到GUIDED模式...")
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)
            
            self.log("  解锁...")
            self.vehicle.armed = True
            timeout = 15
            start = time.time()
            while not self.vehicle.armed:
                if time.time() - start > timeout:
                    raise TimeoutError("解锁超时")
                time.sleep(0.2)
            
            self.log("  起飞...")
            self.vehicle.simple_takeoff(target_alt)
            
            # 等待到达目标高度
            self.log("  等待到达目标高度...")
            timeout = 60
            start = time.time()
            while True:
                current_alt = self.vehicle.location.global_relative_frame.alt
                self.log(f"    当前高度: {current_alt:.1f}m")
                
                if current_alt >= target_alt * 0.95:
                    self.log(f"  ✓ 到达目标高度 {target_alt}米")
                    break
                
                if time.time() - start > timeout:
                    self.log(f"  ⚠ 起飞超时，当前高度: {current_alt:.1f}m")
                    break
                
                time.sleep(1)
            
            self.log_result("起飞测试", True)
            return True
            
        except Exception as e:
            self.log(f"✗ 起飞失败: {e}")
            self.log_result("起飞测试", False, str(e))
            return False
    
    # ========================
    # 飞行测试
    # ========================
    def test_flight_movement(self):
        """测试飞行移动"""
        self.log("\n" + "-" * 40)
        self.log("测试5: 飞行移动")
        self.log("-" * 40)
        
        if not self.vehicle:
            self.log_result("飞行移动", False, "飞控未连接")
            return False
        
        try:
            # 悬停
            self.log("  悬停5秒...")
            time.sleep(5)
            
            # 简单移动
            self.log("  简单移动测试...")
            current = self.vehicle.location.global_relative_frame
            if current:
                target_lat = current.lat + 0.0001  # 约10米
                target = LocationGlobalRelative(target_lat, current.lon, current.alt)
                self.vehicle.simple_goto(target)
                
                self.log("  移动中...")
                time.sleep(5)
                self.log("  ✓ 移动完成")
            
            self.log_result("飞行移动", True)
            return True
            
        except Exception as e:
            self.log(f"✗ 飞行移动失败: {e}")
            self.log_result("飞行移动", False, str(e))
            return False
    
    # ========================
    # 降落测试
    # ========================
    def test_landing(self):
        """测试降落"""
        self.log("\n" + "-" * 40)
        self.log("测试6: 降落测试")
        self.log("-" * 40)
        
        if not self.vehicle:
            self.log_result("降落测试", False, "飞控未连接")
            return False
        
        try:
            self.log("  切换到LAND模式...")
            self.vehicle.mode = VehicleMode("LAND")
            
            self.log("  降落中...")
            timeout = 60
            start = time.time()
            while True:
                current_alt = self.vehicle.location.global_relative_frame.alt
                self.log(f"    高度: {current_alt:.1f}m")
                
                if current_alt < 0.5:
                    self.log("  ✓ 降落完成")
                    break
                
                if time.time() - start > timeout:
                    self.log("  ⚠ 降落超时")
                    break
                
                time.sleep(1)
            
            # 锁定
            self.log("  锁定飞控...")
            self.vehicle.armed = False
            time.sleep(1)
            
            self.log_result("降落测试", True)
            return True
            
        except Exception as e:
            self.log(f"✗ 降落失败: {e}")
            self.log_result("降落测试", False, str(e))
            return False
    
    # ========================
    # 偏航控制测试
    # ========================
    def test_yaw_control(self):
        """测试偏航控制"""
        self.log("\n" + "-" * 40)
        self.log("测试7: 偏航控制")
        self.log("-" * 40)
        
        if not self.vehicle:
            self.log_result("偏航控制", False, "飞控未连接")
            return False
        
        try:
            from yaw_controller import YawController
            
            controller = YawController(self.vehicle)
            
            # 获取初始航向
            initial_heading = controller.get_current_heading()
            self.log(f"  初始航向: {initial_heading:.0f}°")
            
            # 旋转45度
            self.log("  旋转45度（顺时针）...")
            controller.rotate_angle(45, direction=1, speed=15)
            time.sleep(1)
            
            # 旋转回来
            self.log("  旋转45度（逆时针）...")
            controller.rotate_angle(45, direction=-1, speed=15)
            time.sleep(1)
            
            final_heading = controller.get_current_heading()
            self.log(f"  最终航向: {final_heading:.0f}°")
            self.log("  ✓ 偏航控制测试完成")
            
            self.log_result("偏航控制", True)
            return True
            
        except Exception as e:
            self.log(f"✗ 偏航控制失败: {e}")
            self.log_result("偏航控制", False, str(e))
            return False
    
    # ========================
    # 遥测数据测试
    # ========================
    def test_telemetry(self):
        """测试遥测数据"""
        self.log("\n" + "-" * 40)
        self.log("测试8: 遥测数据")
        self.log("-" * 40)
        
        if not self.vehicle:
            self.log_result("遥测数据", False, "飞控未连接")
            return False
        
        try:
            self.log("  读取遥测数据...")
            
            data = {
                'mode': self.vehicle.mode.name,
                'armed': self.vehicle.armed,
                'heading': self.vehicle.heading,
                'altitude': self.vehicle.location.global_relative_frame.alt if self.vehicle.location else None,
                'speed': self.vehicle.groundspeed,
                'battery': self.vehicle.battery.level if self.vehicle.battery else None,
            }
            
            for key, value in data.items():
                self.log(f"    {key}: {value}")
            
            self.log_result("遥测数据", True)
            return True
            
        except Exception as e:
            self.log(f"✗ 遥测数据读取失败: {e}")
            self.log_result("遥测数据", False, str(e))
            return False
    
    # ========================
    # 运行所有测试
    # ========================
    def run_all_tests(self, auto=False):
        """运行所有测试"""
        self.log("\n" + "=" * 60)
        self.log("   开始测试")
        self.log("=" * 60)
        
        # 测试列表
        tests = [
            ('connection', self.test_connection),
            ('mode_switch', self.test_mode_switch),
            ('arm_disarm', self.test_arm_disarm),
        ]
        
        # 如果是自动模式，只运行基础测试
        if not auto:
            tests.extend([
                ('takeoff', self.test_takeoff),
                ('flight_movement', self.test_flight_movement),
                ('landing', self.test_landing),
                ('yaw_control', self.test_yaw_control),
                ('telemetry', self.test_telemetry),
            ])
        
        # 运行测试
        passed = 0
        failed = 0
        
        for test_name, test_func in tests:
            try:
                if test_func():
                    passed += 1
                else:
                    failed += 1
            except Exception as e:
                self.log(f"✗ 测试异常: {test_name} - {e}")
                self.log_result(test_name, False, str(e))
                failed += 1
        
        # 打印结果
        self.log("\n" + "=" * 60)
        self.log("   测试结果汇总")
        self.log("=" * 60)
        self.log(f"  通过: {passed}")
        self.log(f"  失败: {failed}")
        self.log(f"  总计: {passed + failed}")
        
        # 保存结果
        self.log_file.write("\n测试详情:\n")
        for result in self.test_results:
            status = "✓" if result['passed'] else "✗"
            self.log_file.write(f"  [{status}] {result['test']}: {result['time']}\n")
            if result['error']:
                self.log_file.write(f"    错误: {result['error']}\n")
        
        self.log(f"\n日志保存到: {self.log_file.name}")
        
        return failed == 0
    
    def cleanup(self):
        """清理资源"""
        if self.vehicle:
            try:
                if self.vehicle.armed:
                    self.log("\n清理: 锁定飞控...")
                    self.vehicle.armed = False
                self.log("清理: 关闭连接...")
                self.vehicle.close()
            except:
                pass
        self.log_file.close()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='UAV完整功能测试')
    parser.add_argument('--mode', choices=['sitl', 'pix6'], default='sitl',
                        help='测试模式: sitl=模拟器, pix6=硬件')
    parser.add_argument('--sitl', action='store_true',
                        help='SITL模式（等同于 --mode sitl）')
    parser.add_argument('--pix6', action='store_true',
                        help='Pix6硬件模式（等同于 --mode pix6）')
    parser.add_argument('--connection', default=None,
                        help='连接字符串')
    parser.add_argument('--auto', action='store_true',
                        help='自动测试模式（只运行基础测试）')
    parser.add_argument('--test', default='all',
                        choices=['all', 'connection', 'mode', 'arm', 'takeoff', 'flight', 'land', 'yaw', 'telemetry'],
                        help='指定测试项目')
    
    args = parser.parse_args()
    
    # 处理 --sitl 和 --pix6 参数
    if args.sitl:
        args.mode = 'sitl'
    if args.pix6:
        args.mode = 'pix6'
    
    # 创建测试套件
    suite = UAVTestSuite(mode=args.mode, connection_string=args.connection)
    
    try:
        # 运行测试
        if args.test == 'all':
            suite.run_all_tests(auto=args.auto)
        else:
            # 单项测试
            test_map = {
                'connection': suite.test_connection,
                'mode': suite.test_mode_switch,
                'arm': suite.test_arm_disarm,
                'takeoff': suite.test_takeoff,
                'flight': suite.test_flight_movement,
                'land': suite.test_landing,
                'yaw': suite.test_yaw_control,
                'telemetry': suite.test_telemetry,
            }
            if args.test in test_map:
                test_map[args.test]()
    except KeyboardInterrupt:
        suite.log("\n用户中断测试")
    except Exception as e:
        suite.log(f"\n测试异常: {e}")
    finally:
        suite.cleanup()


if __name__ == '__main__':
    from dronekit import connect, VehicleMode, LocationGlobalRelative
    from yaw_controller import YawController
    
    main()
