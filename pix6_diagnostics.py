#!/usr/bin/env python3
"""
Pix6 飞控诊断工具
连接并获取所有参数和遥测数据，验证是否获取真实信息
"""

import sys
import time
from dronekit import connect, VehicleMode

CONN_STRING = "/dev/ttyACM0"
BAUD = 921600

print("=" * 60)
print("  Pix6 飞控诊断工具")
print("=" * 60)
print(f"\n连接飞控: {CONN_STRING} @ {BAUD}")
print("-" * 60)

try:
    print("\n正在连接...")
    vehicle = connect(CONN_STRING, baud=BAUD, wait_ready=True, timeout=30)
    print("✓ 连接成功!\n")
except Exception as e:
    print(f"✗ 连接失败: {e}")
    sys.exit(1)

print("=" * 60)
print("  1. Vehicle对象所有属性")
print("=" * 60)
attrs = [a for a in dir(vehicle) if not a.startswith('_')]
for attr in attrs:
    print(f"  {attr}")

print("\n" + "=" * 60)
print("  2. 当前状态快照")
print("=" * 60)
print(f"  mode.name:           {vehicle.mode.name}")
print(f"  armed:               {vehicle.armed}")
print(f"  heading:             {vehicle.heading}")
print(f"  groundspeed:         {vehicle.groundspeed}")

print("\n" + "=" * 60)
print("  3. 位置信息")
print("=" * 60)
loc = vehicle.location
if loc:
    print(f"  location.global_frame:")
    if loc.global_frame:
        print(f"    lat:  {loc.global_frame.lat}")
        print(f"    lon:  {loc.global_frame.lon}")
        print(f"    alt:  {loc.global_frame.alt}")
    else:
        print("    (None)")

    print(f"  location.global_relative_frame:")
    if loc.global_relative_frame:
        print(f"    lat:  {loc.global_relative_frame.lat}")
        print(f"    lon:  {loc.global_relative_frame.lon}")
        print(f"    alt:  {loc.global_relative_frame.alt}")
    else:
        print("    (None)")

    print(f"  location.local_frame:")
    if loc.local_frame:
        print(f"    north: {loc.local_frame.north}")
        print(f"    east:  {loc.local_frame.east}")
        print(f"    down:  {loc.local_frame.down}")
    else:
        print("    (None)")
else:
    print("  location: None")

print("\n" + "=" * 60)
print("  4. 电池信息")
print("=" * 60)
if vehicle.battery:
    print(f"  voltage:  {vehicle.battery.voltage}")
    print(f"  current:  {vehicle.battery.current}")
    print(f"  level:    {vehicle.battery.level}")
else:
    print("  battery: None")

print("\n" + "=" * 60)
print("  5. GPS相关参数和信息")
print("=" * 60)
params = vehicle.parameters
gps_params = ['GPS_TYPE', 'GPS_TYPE2', 'GPS_ENABLE', 'GPS_RATE_MS', 'GPS_DELAY_MS']
print("  GPS参数:")
for gp in gps_params:
    val = params.get(gp, 'N/A')
    print(f"    {gp}: {val}")

print("\n  GPS状态:")
try:
    gps_raw = vehicle._master.messages.get('GPS_RAW_INT')
    if gps_raw:
        print(f"    fix_type:    {gps_raw.fix_type}")
        print(f"    satellites:  {gps_raw.satellites_visible}")
        print(f"    hdop:        {gps_raw.eph}")
        print(f"    lat:         {gps_raw.lat}")
        print(f"    lon:         {gps_raw.lon}")
        print(f"    alt:         {gps_raw.alt}")
        print(f"    speed:       {gps_raw.vel}")
    else:
        print("    无GPS_RAW_INT数据")
except Exception as e:
    print(f"    获取GPS状态失败: {e}")

print("\n  GPS fix类型说明:")
print("    0: 无GPS")
print("    1: 无定位")
print("    2: 2D定位")
print("    3: 3D定位")
print("    4: DGPS")
print("    5: RTK固定")
print("    6: RTK浮动")

print("\n" + "=" * 60)
print("  6. 所有参数")
print("=" * 60)
params = vehicle.parameters
print(f"  共 {len(params)} 个参数\n")
for key, value in sorted(params.items()):
    print(f"    {key}: {value}")

print("\n" + "=" * 60)
print("  6. 实时监控 (30秒, Ctrl+C退出)")
print("=" * 60)
print("  格式: [时间] 高度(m) | 航向(°) | 纬度, 经度")
print("-" * 60)

try:
    for i in range(30):
        time.sleep(1)
        loc = vehicle.location.global_relative_frame
        alt = loc.alt if loc and loc.alt else 0
        heading = vehicle.heading
        lat = loc.lat if loc and loc.lat else 0
        lon = loc.lon if loc and loc.lon else 0
        print(f"  [{i+1:2d}] {alt:6.2f}m | {heading:3d}° | {lat:.7f}, {lon:.7f}")
except KeyboardInterrupt:
    print("\n  监控已停止")

print("\n" + "=" * 60)
print("  诊断完成")
print("=" * 60)
vehicle.close()
print("✓ 已断开连接")
