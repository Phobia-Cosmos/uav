#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAVLink基础测试脚本

测试飞控连接、起飞、速度控制、航向控制等基本功能。
支持SITL仿真和Pix6硬件两种模式。

用法:
    python3 mavlink_test.py --sitl              # SITL仿真模式
    python3 mavlink_test.py --pix6              # Pix6硬件模式
    python3 mavlink_test.py --sitl --auto       # SITL自动测试
"""

from __future__ import division, print_function
import time
import sys
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


class MAVLinkTest:
    """MAVLink基础测试类"""

    def __init__(self, connection_string, baud=57600):
        """
        初始化测试

        Args:
            connection_string: 连接字符串
            baud: 波特率
        """
        self.connection_string = connection_string
        self.baud = baud
        self.vehicle = None

    def connect(self):
        """连接飞控"""
        print("=" * 60)
        print("   MAVLink基础测试")
        print("=" * 60)
        print("\n连接飞控: %s @ %d" % (self.connection_string, self.baud))

        try:
            self.vehicle = connect(
                self.connection_string,
                wait_ready=True,
                baud=self.baud,
                timeout=30
            )
            print("  连接成功!")
            print("  模式: %s" % self.vehicle.mode.name)
            print("  解锁: %s" % self.vehicle.armed)
            return True
        except Exception as e:
            print("  连接失败: %s" % e)
            return False

    def disconnect(self):
        """断开连接"""
        if self.vehicle:
            self.vehicle.close()
            print("  已断开连接")

    def arm_and_takeoff(self, target_altitude):
        """
        起飞到指定高度

        Args:
            target_altitude: 目标高度（米）
        """
        if not self.vehicle:
            print("错误: 飞控未连接")
            return False

        print("\n[起飞] 目标高度: %dm" % target_altitude)

        # 等待飞控就绪
        while not self.vehicle.is_armable:
            print("  等待飞控初始化...")
            time.sleep(1)

        # 切换到GUIDED模式
        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)

        # 解锁
        print("  解锁电机...")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print("  等待解锁...")
            time.sleep(0.5)

        # 起飞
        print("  起飞中...")
        self.vehicle.simple_takeoff(target_altitude)

        # 等待到达目标高度
        while True:
            altitude = self.vehicle.location.global_relative_frame.alt
            if altitude:
                print("  当前高度: %.1fm" % altitude)

            if altitude >= target_altitude * 0.95:
                print("  到达目标高度")
                return True

            time.sleep(0.5)

    def send_global_ned_velocity(self, vx, vy, vz):
        """
        以全局坐标系发送NED速度

        Args:
            vx: 北向速度 (m/s)
            vy: 东向速度 (m/s)
            vz: 下向速度 (m/s)
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # 时间戳
            0,  # 目标系统
            0,  # 目标组件
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110111000111,  # 位置掩码
            0, 0, 0,  # 位置 (x, y, z)
            vx, vy, vz,  # 速度 (vx, vy, vz)
            0, 0, 0,  # 加速度
            0, 0  # 偏航, 偏航率
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def send_body_ned_velocity(self, vx, vy, vz):
        """
        以机体坐标系发送NED速度

        Args:
            vx: 前进速度 (m/s)
            vy: 右侧速度 (m/s)
            vz: 上升速度 (m/s, 负值为下降)
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # 时间戳
            0,  # 目标系统
            0,  # 目标组件
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,  # 速度掩码
            0, 0, 0,  # 位置 (x, y, z)
            vx, vy, vz,  # 速度 (vx, vy, vz)
            0, 0, 0,  # 加速度
            0, 0  # 偏航, 偏航率
        )

        # 发送一次
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def condition_yaw(self, degrees, direction, relative=True):
        """
        控制航向

        Args:
            degrees: 偏航角度（度）
            direction: 方向 (1=顺时针, -1=逆时针)
            relative: 是否相对当前航向
        """
        is_relative = 1 if relative else 0

        msg = self.vehicle.message_factory.command_long_encode(
            0,  # 目标系统
            0,  # 目标组件
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,  # 确认
            degrees,  # param1: 角度
            0,  # param2: 角速度
            direction,  # param3: 方向
            is_relative,  # param4: 相对/绝对
            0, 0, 0  # 保留
        )

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def land(self):
        """降落"""
        print("\n[降落]")
        self.vehicle.mode = VehicleMode("LAND")

    def run_test_sequence(self):
        """运行测试序列"""
        if not self.connect():
            return

        try:
            # 起飞到2米
            self.arm_and_takeoff(2)
            time.sleep(2)

            # 前进
            print("\n[测试] 前进")
            self.send_body_ned_velocity(0.5, 0, 0)
            time.sleep(3)

            # 右转30度
            print("\n[测试] 右转30度")
            self.condition_yaw(30, 1, relative=True)
            time.sleep(2)

            # 前进
            print("\n[测试] 前进")
            self.send_body_ned_velocity(0.5, 0, 0)
            time.sleep(3)

            # 左转60度
            print("\n[测试] 左转60度")
            self.condition_yaw(60, -1, relative=True)
            time.sleep(2)

            # 后退
            print("\n[测试] 后退")
            self.send_body_ned_velocity(-0.3, 0, 0)
            time.sleep(3)

            # 降落
            print("\n[测试] 降落")
            self.land()
            time.sleep(5)

            # 锁定
            print("\n[测试] 锁定")
            self.vehicle.armed = False

            print("\n测试完成!")

        except KeyboardInterrupt:
            print("\n用户中断")
        finally:
            self.disconnect()


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="MAVLink基础测试脚本"
    )
    parser.add_argument(
        "--sitl",
        action="store_true",
        help="SITL仿真模式 (默认: udp:127.0.0.1:14550)"
    )
    parser.add_argument(
        "--pix6",
        action="store_true",
        help="Pix6硬件模式 (默认: /dev/ttyACM0)"
    )
    parser.add_argument(
        "--connection",
        type=str,
        default=None,
        help="自定义连接字符串"
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=57600,
        help="波特率 (默认: 57600)"
    )

    return parser.parse_args()


def main():
    """主函数"""
    args = parse_args()

    # 确定连接字符串和波特率
    if args.connection:
        connection_string = args.connection
    elif args.pix6:
        connection_string = "/dev/ttyACM0"
        baud = 921600
    else:
        connection_string = "udp:127.0.0.1:14550"
        baud = 57600

    if args.baud != 57600 or args.pix6:
        baud = args.baud
    else:
        baud = 57600

    # 创建测试实例并运行
    test = MAVLinkTest(connection_string, baud)
    test.run_test_sequence()


if __name__ == "__main__":
    main()
