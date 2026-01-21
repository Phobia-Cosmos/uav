#!/usr/bin/env python
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
# 解锁（不使用 GPS）
# ========================
def arm_no_gps():
    # print("Basic pre-arm checks (NO GPS)")

    # # 等待飞控启动完成
    # while not vehicle.is_armable:
    #     print(" Waiting for vehicle to initialise...")
    #     time.sleep(1)

    # # 室内模式：推荐 STABILIZE 或 ALT_HOLD
    # print("Setting mode to STABILIZE")
    # vehicle.mode = VehicleMode("STABILIZE")

    # while vehicle.mode.name != "STABILIZE":
    #     print(" Waiting for mode change...")
    #     time.sleep(0.5)

    # print("Arming motors")
    # vehicle.armed = True

    # while not vehicle.armed:
    #     print(" Waiting for arming...")
    #     time.sleep(0.5)

    # print("Vehicle armed (NO GPS)")
    print("Waiting for vehicle to initialise...")
    time.sleep(5)

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
            raise RuntimeError("Arming failed")
        time.sleep(0.2)

    print("Vehicle armed")



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
        while vehicle.armed:
            time.sleep(0.2)

    print("Closing vehicle")
    vehicle.close()

print("Done")
