#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
转向控制扩展模块
提供持续旋转、角度旋转和完整旋转机动功能
"""
import time
from pymavlink import mavutil


class YawController:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def rotate_continuous(self, direction=1, speed=10, duration=None):
        if not self.vehicle:
            print("错误: 飞控未连接")
            return False

        if direction not in [1, -1]:
            print("错误: direction必须为1(顺时针)或-1(逆时针)")
            return False

        if speed <= 0:
            print("错误: speed必须大于0")
            return False

        print(f"开始持续旋转: 方向={direction}, 速度={speed}度/秒")

        try:
            start_time = time.time()

            while True:
                msg = self.vehicle.message_factory.command_long_encode(
                    0, 0,
                    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                    0,
                    0,
                    speed,
                    direction,
                    1,
                    0, 0, 0
                )
                self.vehicle.send_mavlink(msg)
                self.vehicle.flush()

                if duration and (time.time() - start_time) >= duration:
                    print(f"持续旋转完成 ({duration}秒)")
                    break

                time.sleep(0.1)

            return True

        except Exception as e:
            print(f"持续旋转失败: {e}")
            return False

    def rotate_angle(self, angle, direction=1, speed=15, relative=True):
        if not self.vehicle:
            print("错误: 飞控未连接")
            return False

        if angle <= 0:
            print("错误: angle必须大于0")
            return False

        if direction not in [1, -1]:
            print("错误: direction必须为1(顺时针)或-1(逆时针)")
            return False

        if speed <= 0:
            print("错误: speed必须大于0")
            return False

        direction_name = "顺时针" if direction == 1 else "逆时针"
        print(f"旋转: {angle}度, {direction_name}, 速度={speed}度/秒")

        try:
            is_relative = 1 if relative else 0

            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                angle,
                speed,
                direction,
                is_relative,
                0, 0, 0
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()

            time.sleep(abs(angle) / speed + 1)

            print(f"旋转完成")
            return True

        except Exception as e:
            print(f"角度旋转失败: {e}")
            return False

    def stop_rotation(self):
        try:
            self.vehicle.channels.overrides = {}
            print("旋转已停止")
            return True
        except Exception as e:
            print(f"停止旋转失败: {e}")
            return False

    def get_current_heading(self):
        try:
            if hasattr(self.vehicle, 'heading'):
                return self.vehicle.heading or 0
            return 0
        except Exception:
            return 0

    def get_attitude(self):
        try:
            if self.vehicle.attitude:
                return self.vehicle.attitude
            return None
        except Exception:
            return None


def create_yaw_controller(vehicle):
    return YawController(vehicle)


def run_test(connection_string='tcp:127.0.0.1:5760', baud=57600, auto_test=False, test_angle=45):
    """运行转向控制器测试

    Args:
        connection_string: 连接字符串
        baud: 波特率
        auto_test: 是否自动运行测试
        test_angle: 自动测试时的旋转角度
    """
    from dronekit import connect, VehicleMode

    print("=" * 60)
    print("   转向控制器测试程序")
    print("=" * 60)
    print(f"连接: {connection_string} @ {baud}")
    print("-" * 60)

    vehicle = None
    try:
        vehicle = connect(connection_string, wait_ready=True, baud=baud)
        print("✓ 飞控连接成功!")
        print(f"  模式: {vehicle.mode.name}")
        print(f"  姿态: pitch={vehicle.attitude.pitch:.2f}, roll={vehicle.attitude.roll:.2f}, yaw={vehicle.attitude.yaw:.2f}")
        print("-" * 60)

        controller = YawController(vehicle)

        if auto_test:
            # 自动测试模式
            print("\n[自动测试模式]")
            print("-" * 60)

            # 测试1: 查看姿态
            print("\n[1] 查看当前姿态")
            att = vehicle.attitude
            print(f"  Pitch: {att.pitch:.2f}°")
            print(f"  Roll:  {att.roll:.2f}°")
            print(f"  Yaw:   {att.yaw:.2f}°")

            # 测试2: 旋转测试
            print(f"\n[2] 旋转{test_angle}度（顺时针）")
            controller.rotate_angle(test_angle, direction=1, speed=15)
            time.sleep(1)

            print(f"\n[3] 旋转{test_angle}度（逆时针）")
            controller.rotate_angle(test_angle, direction=-1, speed=15)
            time.sleep(1)

            # 测试3: 获取航向
            print(f"\n[4] 当前航向: {controller.get_current_heading():.0f}°")

            print("\n" + "=" * 60)
            print("自动测试完成!")
            print("=" * 60)

        else:
            # 交互式测试
            while True:
                print("\n可选测试:")
                print("  1. 查看当前姿态")
                print("  2. 获取航向")
                print("  3. 旋转45度（顺时针）")
                print("  4. 旋转45度（逆时针）")
                print("  5. 自定义角度旋转")
                print("  6. 持续旋转3秒")
                print("  7. 系统自检")
                print("  0. 退出测试")
                print("-" * 60)

                choice = input("请选择 [0-7]: ").strip()

                if choice == '1':
                    att = vehicle.attitude
                    if att:
                        print(f"\n  Pitch: {att.pitch:.2f}°")
                        print(f"  Roll:  {att.roll:.2f}°")
                        print(f"  Yaw:   {att.yaw:.2f}°")

                elif choice == '2':
                    print(f"\n当前航向: {controller.get_current_heading():.0f}°")

                elif choice == '3':
                    print("\n旋转45度（顺时针）...")
                    controller.rotate_angle(45, direction=1, speed=15)

                elif choice == '4':
                    print("\n旋转45度（逆时针）...")
                    controller.rotate_angle(45, direction=-1, speed=15)

                elif choice == '5':
                    try:
                        angle = float(input("输入旋转角度: ").strip())
                        direction = input("方向 (1=顺时针, -1=逆时针): ").strip()
                        direction = int(direction) if direction else 1
                        print(f"\n旋转{angle}度...")
                        controller.rotate_angle(angle, direction=direction, speed=15)
                    except:
                        print("输入无效")

                elif choice == '6':
                    print("\n持续旋转3秒...")
                    controller.rotate_continuous(direction=1, speed=10, duration=3)

                elif choice == '7':
                    print("\n系统自检...")
                    print(f"  连接: ✓")
                    print(f"  模式: {vehicle.mode.name}")
                    att = vehicle.attitude
                    if att:
                        print(f"  姿态: pitch={att.pitch:.2f}, roll={att.roll:.2f}, yaw={att.yaw:.2f}")

                elif choice == '0':
                    print("\n退出测试")
                    break

                else:
                    print("无效选择")

    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        if vehicle:
            vehicle.close()
            print("\n飞控连接已关闭")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='转向控制器测试')
    parser.add_argument('--connection', default='tcp:127.0.0.1:5760',
                        help='飞控连接串 (SITL默认: tcp:127.0.0.1:5760)')
    parser.add_argument('--baud', type=int, default=57600, help='波特率 (SITL默认: 57600)')
    parser.add_argument('--auto', action='store_true', help='自动测试模式')
    parser.add_argument('--angle', type=float, default=45, help='自动测试时的旋转角度')

    args = parser.parse_args()

    run_test(args.connection, args.baud, args.auto, args.angle)
