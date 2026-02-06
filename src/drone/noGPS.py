#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import sys
from dronekit import connect, VehicleMode

# ========================
# 连接配置
# ========================
connection_string = '/dev/ttyACM0'
baudrate = 921600

print("Connecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=baudrate)

# ========================
# 禁用RC和失控保护
# ========================
def disable_rc_requirement():
    print("Disabling RC and failsafe requirements...")
    
    params_to_set = [
        ('BRD_RC_TYPES', 0),      # 禁用RC类型检测
        ('FS_THR_ENABLE', 0),     # 禁用油门失控保护
        ('ARMING_CHECK', 0),      # 禁用所有检查（谨慎使用）
    ]
    
    for param_name, value in params_to_set:
        try:
            vehicle.parameters[param_name] = value
            print(f"  {param_name} = {value} ✓")
            time.sleep(0.5)
        except Exception as e:
            print(f"  {param_name} 设置失败: {e}")
    
    print("  注意: 如果仍失败，需要通过MAVLink手动设置参数")

# ========================
# 解锁（不使用 GPS）
# ========================
def arm_no_gps():
    print("Waiting for vehicle to initialise...")
    time.sleep(1)

    # 禁用RC检测
    disable_rc_requirement()
    time.sleep(1)

    print("Setting mode to STABILIZE")
    vehicle.mode = VehicleMode("STABILIZE")
    while vehicle.mode.name != "STABILIZE":
        time.sleep(0.2)

    print("Arming motors (NO is_armable check)")
    vehicle.armed = True

    timeout = 10
    start = time.time()
    while not vehicle.armed:
        if time.time() - start > timeout:
            raise RuntimeError("Arming failed - RC receiver may still be required")
        time.sleep(0.2)

    print("Vehicle armed!")



# ========================
# 主流程
# ========================
try:
    arm_no_gps()

    # print("Motors armed, running for 20 seconds")
    # time.sleep(20)
    print("\nStarting throttle ramp test (RC override)")
    print("Press Ctrl+C to stop\n")

    throttle_min = 1050   # 刚过 idle
    throttle_max = 1250   # 明显变快，但远离起飞区
    throttle_step = 5     # 每次增加 5
    period = 0.1          # 10 Hz

    throttle = throttle_min
    direction = 1

    while True:
        vehicle.channels.overrides = {
            '3': throttle   # 通道 3 = 油门
        }

        print("Throttle PWM:", throttle)

        throttle += direction * throttle_step

        if throttle >= throttle_max:
            direction = -1
        elif throttle <= throttle_min:
            direction = 1

        time.sleep(period)

except KeyboardInterrupt:
    print("\nCtrl+C detected!")
    print("\n--- 调试信息 ---")
    try:
        mode_name = vehicle.mode.name if vehicle.mode else "未知"
    except:
        mode_name = "获取失败"
    print(f"  飞控模式: {mode_name}")
    print(f"  解锁状态: {vehicle.armed}")
    try:
        print(f"  系统状态: {vehicle.system_status.state}")
    except:
        print("  系统状态: 未知")
    try:
        print(f"  RC通道: {vehicle.channels}")
    except:
        print("  RC通道: 未知")
    try:
        print(f"  油门override: {vehicle.channels.overrides}")
    except:
        print("  油门override: 未知")
    
    # 显示飞控日志中的错误
    print("\n检查飞控是否响应...")
    
    # 多次尝试清除override
    print("清除RC override...")
    vehicle.channels.overrides = {}
    
    # 等待飞控处理清除命令
    print("等待飞控处理...")
    time.sleep(2)
    
    # 检查override是否真的被清除
    print(f"当前override: {vehicle.channels.overrides}")
    
    # 使用MAVLink命令强制解锁
    print("发送MAVLink强制解锁命令...")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,       # target system, component
        400,        # MAV_CMD_COMPONENT_ARM_DISARM
        0,          # confirmation
        0,          # param1: 0=disarm
        21196,      # param2: force=21196
        0, 0, 0, 0, 0  # params
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    time.sleep(2)
    
    print(f"解锁状态: {vehicle.armed}")
    
    if vehicle.armed:
        print("\n飞控仍未响应！可能需要重启飞控")
        print("请手动断开USB，然后重新连接")

finally:
    # print("Disarming motors")
    # if vehicle.armed:
    #     vehicle.armed = False
    #     while vehicle.armed:
    #         print(" Waiting for disarm...")
    #         time.sleep(0.5)

    # print("Closing vehicle")
    # vehicle.close()
    print("Clearing RC override")
    vehicle.channels.overrides = {}

    print("Disarming motors")
    if vehicle.armed:
        vehicle.armed = False
        timeout = 5
        start = time.time()
        while vehicle.armed:
            if time.time() - start > timeout:
                print("  强制关闭连接...")
                break
            time.sleep(0.2)

    print("Closing vehicle")
    vehicle.close()

print("Done")
