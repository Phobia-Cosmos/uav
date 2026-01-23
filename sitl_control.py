#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SITL模拟器启动脚本
"""
import subprocess
import time
import os

def start_sitl():
    """启动SITL模拟器"""
    ardupilot_dir = "/home/undefined/Desktop/uav/ardupilot"

    print("=" * 60)
    print("   启动 SITL 模拟器")
    print("=" * 60)

    # 检查是否已在运行
    result = subprocess.run(
        "ps aux | grep -E 'arducopter|sim_vehicle' | grep -v grep | grep -v python",
        shell=True, capture_output=True, text=True
    )
    if result.stdout.strip():
        print("SITL已在运行!")
        return True

    # 启动SITL
    print("\n启动SITL中...")
    log_file = open("/tmp/sitl.log", "w")

    process = subprocess.Popen(
        ["./build/sitl/bin/arducopter",
         "--model", "+",
         "--speedup", "1",
         "--slave", "0",
         "--defaults", "Tools/autotest/default_params/copter.parm",
         "--sim-address=127.0.0.1",
         "-I0"],
        cwd=ardupilot_dir,
        stdout=log_file,
        stderr=subprocess.STDOUT
    )

    # 等待启动
    print("等待SITL启动...")
    for i in range(30):
        time.sleep(1)
        # 检查端口
        result = subprocess.run(
            "ss -tlnp 2>/dev/null | grep 5760 || netstat -tlnp 2>/dev/null | grep 5760",
            shell=True, capture_output=True, text=True
        )
        if result.stdout.strip():
            print("✓ SITL启动成功!")
            print("  TCP端口: 5760 (MavProxy连接)")
            print("  UDP端口: 14550 (可用于dronekit)")
            print("\n连接方式:")
            print("  MavProxy: mavproxy.py --master=tcp:127.0.0.1:5760")
            print("  DroneKit: udp:127.0.0.1:14550")
            print("\n按 Ctrl+C 停止SITL")
            return True

        if i % 5 == 0:
            print(f"  等待中... ({i+1}s)")

    print("✗ SITL启动超时")
    return False

def stop_sitl():
    """停止SITL"""
    print("\n停止SITL...")
    subprocess.run("pkill -9 -f arducopter", shell=True)
    subprocess.run("pkill -9 -f sim_vehicle", shell=True)
    print("已停止")

def status_sitl():
    """检查SITL状态"""
    result = subprocess.run(
        "ps aux | grep -E 'arducopter' | grep -v grep",
        shell=True, capture_output=True, text=True
    )
    if result.stdout.strip():
        print("✓ SITL正在运行")
        subprocess.run("ss -tlnp | grep -E '5760|1455'", shell=True)
    else:
        print("✗ SITL未运行")

if __name__ == '__main__':
    import sys

    if len(sys.argv) > 1:
        if sys.argv[1] == '--stop':
            stop_sitl()
        elif sys.argv[1] == '--status':
            status_sitl()
        else:
            print("用法: python3 sitl_control.py [--stop|--status]")
    else:
        try:
            start_sitl()
        except KeyboardInterrupt:
            print("\n用户中断")
            stop_sitl()
