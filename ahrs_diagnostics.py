#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AHRS诊断程序
检测和解决"Bad AHRS"及无法解锁电机的问题
"""
import time
import sys
from dronekit import connect, VehicleMode
from pymavlink import mavutil


class AHRSDiagnostics:
    def __init__(self, connection_string='/dev/ttyACM0', baud=921600):
        self.connection_string = connection_string
        self.baud = baud
        # vehicle是在连接成功在赋值
        self.vehicle = None
        self.issues = []
        self.solutions = []

    def connect(self):
        """连接飞控"""
        print("=" * 70)
        print("   AHRS诊断程序 - 检测Bad AHRS和无法解锁问题")
        print("=" * 70)
        print(f"\n连接飞控: {self.connection_string} @ {self.baud}")

        try:
            self.vehicle = connect(self.connection_string, wait_ready=True, baud=self.baud, timeout=30)
            print("✓ 飞控连接成功!")
            return True
        except Exception as e:
            print(f"✗ 飞控连接失败: {e}")
            return False

    def check_system_status(self):
        """检查系统状态"""
        print("\n" + "=" * 50)
        print("1. 系统状态检查")
        print("=" * 50)

        try:
            status = self.vehicle.system_status.state
            print(f"   系统状态: {status}")

            if status == 'STANDBY':
                print("   ✓ 飞控已就绪")
            elif status == 'UNINIT':
                print("   ✗ 系统未初始化")
                self.issues.append("系统未初始化")
                self.solutions.append("等待飞控完成初始化（约30-60秒）")
            else:
                # TODO：这里会怎么样？
                print(f"   状态: {status}")
        except Exception as e:
            print(f"   错误: {e}")

    def check_armable_state(self):
        """检查可解锁状态"""
        print("\n" + "=" * 50)
        print("2. 可解锁状态检查 (is_armable)")
        print("=" * 50)

        is_armable = self.vehicle.is_armable
        print(f"   is_armable: {is_armable}")

        if is_armable:
            print("   ✓ 飞控可解锁")
        else:
            print("   ✗ 飞控暂不可解锁")
            self.issues.append("飞控不可解锁")

            if self.vehicle.mode.name == 'STABILIZE':
                print("   提示: 尝试切换到STABILIZE模式后解锁")

    def check_ahrs_health(self):
        """检查AHRS健康状态"""
        print("\n" + "=" * 50)
        print("3. AHRS健康状态检查")
        print("=" * 50)

        try:
            # TODO：这个是什么函数？有什么作用？这个不是dronekit的api吧？除了可以接受heartbeat还可以接受什么值？
            heartbeat = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if heartbeat:
                print(f"   System: {heartbeat.system_status}")

            ahrs = self.vehicle.recv_match(type='AHRS2', blocking=True, timeout=5)
            if ahrs:
                print(f"   AHRS2接收正常")
                print(f"   Roll: {ahrs.roll:.2f}°")
                print(f"   Pitch: {ahrs.pitch:.2f}°")
                print(f"   Yaw: {ahrs.yaw:.2f}°")

                if abs(ahrs.roll) > 90 or abs(ahrs.pitch) > 90:
                    print("   ✗ 姿态异常（角度超出范围）")
                    self.issues.append("AHRS姿态数据异常")
                    self.solutions.append("检查飞控是否水平放置，重置IMU")
                else:
                    print("   ✓ 姿态数据正常")
            else:
                print("   ✗ 未收到AHRS2数据")
                self.issues.append("未收到AHRS数据")
                self.solutions.append("检查IMU连接，重启飞控")

            ahrs3 = self.vehicle.recv_match(type='AHRS3', blocking=True, timeout=3)
            if ahrs3:
                print(f"   AHRS3接收正常")
            else:
                print("   (AHRS3未使用)")

        except Exception as e:
            print(f"   AHRS检查错误: {e}")
            self.issues.append(f"AHRS检查异常: {e}")

    def check_imu_data(self):
        """检查IMU数据"""
        print("\n" + "=" * 50)
        print("4. IMU数据检查")
        print("=" * 50)

        try:
            imu = self.vehicle.recv_match(type='RAW_IMU', blocking=True, timeout=5)
            if imu:
                print(f"   RAW_IMU接收正常")
                print(f"   AccX: {imu.xacc} AccY: {imu.yacc} AccZ: {imu.zacc}")
                print(f"   GyroX: {imu.xgyro} GyroY: {imu.ygyro} GyroZ: {imu.zgyro}")

                acc_magnitude = (imu.xacc**2 + imu.yacc**2 + imu.zacc**2)**0.5
                print(f"   加速度幅值: {acc_magnitude}")

                if 8000 < acc_magnitude < 12000:
                    print("   ✓ 加速度计数据正常（接近1g）")
                elif acc_magnitude < 1000:
                    print("   ✗ 加速度计数据异常（过低，可能未校准）")
                    self.issues.append("加速度计数据异常")
                    self.solutions.append("重新校准加速度计")
                else:
                    print(f"   ⚠ 加速度计数据异常（幅值={acc_magnitude}）")
                    self.issues.append("加速度计可能未校准")
                    self.solutions.append("重新校准加速度计")

            else:
                print("   ✗ 未收到RAW_IMU数据")
                self.issues.append("IMU数据丢失")
                self.solutions.append("检查IMU硬件连接")

        except Exception as e:
            print(f"   IMU检查错误: {e}")

    def check_attitude(self):
        """检查姿态数据"""
        print("\n" + "=" * 50)
        print("5. 姿态数据检查")
        print("=" * 50)

        att = self.vehicle.attitude
        if att:
            # TODO：为什么这里需要*57？
            print(f"   Pitch: {att.pitch:.2f}° ({att.pitch*57.2958:.2f}°)")
            print(f"   Roll:  {att.roll:.2f}° ({att.roll*57.2958:.2f}°)")
            print(f"   Yaw:   {att.yaw:.2f}° ({att.yaw*57.2958:.2f}°)")

            pitch_deg = att.pitch * 57.2958
            roll_deg = att.roll * 57.2958

            if abs(pitch_deg) > 45:
                print(f"   ✗ Pitch角度过大（{pitch_deg:.1f}°）")
                self.issues.append("Pitch角度过大")
                self.solutions.append("确保飞控水平放置")

            if abs(roll_deg) > 45:
                print(f"   ✗ Roll角度过大（{roll_deg:.1f}°）")
                self.issues.append("Roll角度过大")
                self.solutions.append("确保飞控水平放置")

            if abs(pitch_deg) < 5 and abs(roll_deg) < 5:
                print("   ✓ 姿态正常（飞控水平）")
            else:
                print("   ⚠ 飞控可能不在水平位置")

        else:
            print("   ✗ 无姿态数据")
            self.issues.append("无姿态数据")
            self.solutions.append("重启飞控")

    def check_battery(self):
        """检查电池状态"""
        print("\n" + "=" * 50)
        print("6. 电池状态检查")
        print("=" * 50)

        try:
            battery = self.vehicle.battery
            voltage = getattr(battery, 'voltage', 0)
            level = getattr(battery, 'level', 0)

            print(f"   电压: {voltage:.2f}V")
            print(f"   电量: {level}%")

            if voltage > 10.5:
                print("   ✓ 电压正常")
            elif voltage > 9.0:
                print("   ⚠ 电压偏低，可能影响解锁")
                self.issues.append("电池电压偏低")
                self.solutions.append("充电或更换电池")
            else:
                print("   ✗ 电压过低，无法解锁")
                self.issues.append("电池电压过低")
                self.solutions.append("立即充电")

        except Exception as e:
            print(f"   电池检查错误: {e}")

    def check_gps(self):
        """检查GPS状态"""
        print("\n" + "=" * 50)
        print("7. GPS状态检查")
        print("=" * 50)

        try:
            # TODO：将接受的GPS所有信息全部打印出来
            gps = self.vehicle.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
            if gps:
                fix_type = gps.fix_type
                satellites = gps.satellites_visible

                print(f"   GPS类型: {fix_type}")
                print(f"   卫星数量: {satellites}")

                if fix_type == 3:
                    print("   ✓ GPS 3D定位")
                elif fix_type == 2:
                    print("   ⚠ GPS 2D定位（可能在室内）")
                    self.issues.append("GPS信号弱")
                    self.solutions.append("到室外测试，GPS需要开阔天空")
                else:
                    print("   ✗ GPS未定位")
                    self.issues.append("GPS未定位")
                    self.solutions.append("到室外等待GPS锁定")

            else:
                print("   未收到GPS数据")

        except Exception as e:
            print(f"   GPS检查错误: {e}")

    def check_magnetometer(self):
        """检查磁力计状态"""
        print("\n" + "=" * 50)
        print("8. 磁力计状态检查")
        print("=" * 50)

        try:
            mag = self.vehicle.recv_match(type='RAW_IMU', blocking=True, timeout=3)
            if mag:
                print(f"   MagX: {mag.xmag}")
                print(f"   MagY: {mag.ymag}")
                print(f"   MagZ: {mag.zmag}")

                mag_magnitude = (mag.xmag**2 + mag.ymag**2 + mag.zmag**2)**0.5
                print(f"   磁力计幅值: {mag_magnitude}")

                if 300 < mag_magnitude < 2000:
                    print("   ✓ 磁力计数据正常")
                elif mag_magnitude < 100:
                    print("   ✗ 磁力计数据过低（可能未连接或受干扰）")
                    self.issues.append("磁力计异常")
                    self.solutions.append("检查磁力计连接，远离磁场干扰")
                else:
                    print("   ⚠ 磁力计数据异常")
                    self.issues.append("磁力计数据异常")
                    self.solutions.append("重新校准罗盘")

        except Exception as e:
            print(f"   磁力计检查错误: {e}")

    def check_prearm_status(self):
        """检查PreArm状态"""
        print("\n" + "=" * 50)
        print("9. PreArm/安全检查状态")
        print("=" * 50)

        try:
            status = self.vehicle.system_status.state
            print(f"   系统状态: {status}")

            if status == 'STANDBY':
                print("   ✓ 飞控已准备好解锁")
            elif status == 'CALIBRATING':
                print("   正在校准中...")
                self.solutions.append("等待校准完成")
            elif status == 'UNINIT':
                print("   ✗ 系统未初始化")
                self.solutions.append("重启飞控，等待完全启动")
            else:
                print(f"   状态详情: {status}")

            print(f"\n   飞行模式: {self.vehicle.mode.name}")
            print(f"   能否解锁: {self.vehicle.is_armable}")

        except Exception as e:
            print(f"   PreArm检查错误: {e}")

    def try_arm(self):
        """尝试解锁"""
        print("\n" + "=" * 50)
        print("10. 解锁测试")
        print("=" * 50)

        if not self.vehicle.is_armable:
            print("   ✗ 飞控不可解锁，跳过解锁测试")
            print("   可能原因:")
            print("   - 传感器未就绪")
            print("   - PreArm检查未通过")
            print("   - 电池电压低")
            print("   - 飞控不在水平位置")
            return False

        print("   尝试解锁...")

        try:
            self.vehicle.mode = VehicleMode("STABILIZE")
            time.sleep(1)

            self.vehicle.armed = True
            time.sleep(2)

            if self.vehicle.armed:
                print("   ✓ 解锁成功!")
                self.vehicle.armed = False
                time.sleep(1)
                print("   已上锁")
                return True
            else:
                print("   ✗ 解锁失败")
                self.issues.append("解锁失败")
                self.solutions.append("检查上述诊断结果，解决问题后重试")
                return False

        except Exception as e:
            print(f"   解锁错误: {e}")
            self.issues.append(f"解锁异常: {e}")
            return False

    def suggest_solutions(self):
        """提供解决方案建议"""
        print("\n" + "=" * 70)
        print("   问题诊断结果和解决方案")
        print("=" * 70)

        if not self.issues:
            print("\n✓ 未发现明显问题")
            print("\n如果仍然无法解锁，请尝试:")
            print("1. 重启Mission Planner")
            print("2. 重新连接飞控USB线")
            print("3. 重启飞控（断电后重新上电）")
            return

        print(f"\n发现 {len(self.issues)} 个问题:")

        for i, (issue, solution) in enumerate(zip(self.issues, self.solutions), 1):
            print(f"\n问题 {i}: {issue}")
            print(f"建议: {solution}")

        print("\n" + "-" * 70)
        print("通用解决方案:")
        print("-" * 70)
        print("1. 重新校准传感器:")
        print("   - Mission Planner → 初始设置 → 必要硬件 → 重新校准加速度计")
        print("   - Mission Planner → 初始设置 → 必要硬件 → 重新校准罗盘")
        print("")
        print("2. 检查飞控位置:")
        print("   - 确保飞控水平放置")
        print("   - 远离强磁场（大电流、金属物体）")
        print("   - 远离高频振动源")
        print("")
        print("3. 检查电源:")
        print("   - 确保电池电压 > 10.5V")
        print("   - 使用稳定的电源供应")
        print("")
        print("4. 重启飞控:")
        print("   - 断开所有连接")
        print("   - 等待30秒")
        print("   - 重新连接并等待完全初始化")
        print("")
        print("5. 检查连接:")
        print("   - USB线连接稳固")
        print("   - 尝试不同的USB端口")
        print("   - 检查波特率设置正确")
        print("")
        print("6. 更新固件:")
        print("   - 使用Mission Planner重新刷写固件")

    def run_full_diagnostics(self):
        """运行完整诊断"""
        if not self.connect():
            return False

        self.check_system_status()
        self.check_armable_state()
        self.check_ahrs_health()
        self.check_imu_data()
        self.check_attitude()
        self.check_battery()
        self.check_gps()
        self.check_magnetometer()
        self.check_prearm_status()
        self.try_arm()
        self.suggest_solutions()

        if self.vehicle:
            try:
                self.vehicle.close()
            except:
                pass

        print("\n" + "=" * 70)
        print("   诊断完成")
        print("=" * 70)
        return True


def quick_check(connection_string='/dev/ttyACM0', baud=921600):
    """快速检查AHRS状态"""
    print("快速AHRS状态检查...")

    try:
        vehicle = connect(connection_string, wait_ready=True, baud=baud, timeout=15)

        att = vehicle.attitude
        if att:
            print(f"姿态: pitch={att.pitch*57.2958:.1f}°, roll={att.roll*57.2958:.1f}°, yaw={att.yaw*57.2958:.1f}°")

            if abs(att.pitch*57.2958) < 45 and abs(att.roll*57.2958) < 45:
                print("✓ 姿态正常")
            else:
                print("✗ 姿态异常")

        print(f"可解锁: {vehicle.is_armable}")
        print(f"系统状态: {vehicle.system_status.state}")

        vehicle.close()
        return True

    except Exception as e:
        print(f"错误: {e}")
        return False


def main():
    import argparse

    parser = argparse.ArgumentParser(description='AHRS诊断程序')
    parser.add_argument('--connection', default='/dev/ttyACM0',
                        help='飞控连接串 (例如: /dev/ttyACM0 或 udp:127.0.0.1:14550)')
    parser.add_argument('--baud', type=int, default=921600, help='波特率')
    parser.add_argument('--mode', default='full',
                        choices=['full', 'quick'],
                        help='诊断模式')

    args = parser.parse_args()

    diagnostics = AHRSDiagnostics(args.connection, args.baud)

    if args.mode == 'quick':
        quick_check(args.connection, args.baud)
    else:
        diagnostics.run_full_diagnostics()


if __name__ == '__main__':
    main()
