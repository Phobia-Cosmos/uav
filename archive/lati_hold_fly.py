# from dronekit import connect, VehicleMode
# import time

# # =====================
# # 连接飞控
# # =====================
# vehicle = connect('/dev/ttyACM0', baud=921600, wait_ready=True)

# def set_rc(roll=1500, pitch=1500, throttle=1500, yaw=1500):
#     """
#     模拟遥控器打杆
#     """
#     vehicle.channels.overrides = {
#         '1': roll,
#         '2': pitch,
#         '3': throttle,
#         '4': yaw
#     }

# def hold(seconds, **rc):
#     """
#     在给定 RC 输入下保持一段时间（持续发送，防止 failsafe）
#     """
#     start = time.time()
#     while time.time() - start < seconds:
#         set_rc(**rc)
#         time.sleep(0.1)

# print("Setting ALT_HOLD mode")
# vehicle.mode = VehicleMode("ALT_HOLD")
# while vehicle.mode.name != "ALT_HOLD":
#     time.sleep(0.2)

# print("Arming motors")
# vehicle.armed = True
# while not vehicle.armed:
#     time.sleep(0.2)

# # =====================
# # 1. 起飞：缓慢拉油门
# # =====================
# print("Taking off slightly")

# # ALT_HOLD 中：
# # 1500 ≈ 悬停油门
# # 1550~1600 ≈ 缓慢上升
# hold(
#     seconds=3,
#     throttle=1580,   # 轻微上升
#     pitch=1500,
#     roll=1500,
#     yaw=1500
# )

# # 回到悬停油门
# hold(
#     seconds=2,
#     throttle=1500,
#     pitch=1500,
#     roll=1500,
#     yaw=1500
# )

# # =====================
# # 2. 向前飞一点
# # =====================
# print("Flying forward slightly")

# hold(
#     seconds=2,
#     throttle=1500,   # 保持高度
#     pitch=1600,      # 向前倾
#     roll=1500,
#     yaw=1500
# )

# # =====================
# # 3. 悬停
# # =====================
# print("Hovering")

# hold(
#     seconds=3,
#     throttle=1500,
#     pitch=1500,
#     roll=1500,
#     yaw=1500
# )

# # =====================
# # 4. 交出控制权
# # =====================

# print("Releasing control to RC")

# vehicle.channels.overrides = {}

# print("You can now control the vehicle with the transmitter")

# # 不自动上锁，留给遥控器
# time.sleep(5)

# vehicle.close()
# print("Done")
from dronekit import connect, VehicleMode
import time

# =====================
# 连接飞控
# =====================
vehicle = connect('/dev/ttyACM0', baud=921600, wait_ready=True)

def set_rc(roll=1500, pitch=1500, throttle=1500, yaw=1500):
    """
    模拟遥控器打杆
    """
    vehicle.channels.overrides = {
        '1': roll,
        '2': pitch,
        '3': throttle,
        '4': yaw
    }

def hold_with_monitor(seconds, **rc):
    """
    在给定 RC 输入下保持一段时间，同时每秒输出通道状态
    """
    start = time.time()
    last_print = 0
    while time.time() - start < seconds:
        # 持续发送 RC override
        set_rc(**rc)

        # 每隔 1 秒打印一次
        if time.time() - last_print >= 1.0:
            throttle_val = vehicle.channels.overrides.get('3', 'None')
            pitch_val = vehicle.channels.overrides.get('2', 'None')
            roll_val = vehicle.channels.overrides.get('1', 'None')
            yaw_val = vehicle.channels.overrides.get('4', 'None')
            print(f"[RC Override] Throttle={throttle_val}, Pitch={pitch_val}, Roll={roll_val}, Yaw={yaw_val}")
            last_print = time.time()

        time.sleep(0.1)  # 频率 10 Hz

# =====================
# 设置模式 & 解锁
# =====================
print("Setting ALT_HOLD mode")
vehicle.mode = VehicleMode("ALT_HOLD")
while vehicle.mode.name != "ALT_HOLD":
    time.sleep(0.2)

print("Arming motors")
vehicle.armed = True
while not vehicle.armed:
    time.sleep(0.2)

# =====================
# 起飞：缓慢升高
# =====================
print("Taking off slightly")
hold_with_monitor(
    seconds=3,
    throttle=1580,   # 轻微上升
    pitch=1500,
    roll=1500,
    yaw=1500
)

# 回到悬停
hold_with_monitor(
    seconds=2,
    throttle=1500,
    pitch=1500,
    roll=1500,
    yaw=1500
)

# =====================
# 向前飞
# =====================
print("Flying forward slightly")
hold_with_monitor(
    seconds=2,
    throttle=1500,
    pitch=1600,      # 向前
    roll=1500,
    yaw=1500
)

# =====================
# 悬停
# =====================
print("Hovering")
hold_with_monitor(
    seconds=3,
    throttle=1500,
    pitch=1500,
    roll=1500,
    yaw=1500
)

# =====================
# 交出控制权
# =====================
print("Releasing control to RC")
vehicle.channels.overrides = {}

print("Now monitoring channels until RC disarms the vehicle...")

# 持续监控直到遥控器上锁（disarm）
while vehicle.armed:
    throttle_val = vehicle.channels.overrides.get('3', 'None')
    pitch_val = vehicle.channels.overrides.get('2', 'None')
    roll_val = vehicle.channels.overrides.get('1', 'None')
    yaw_val = vehicle.channels.overrides.get('4', 'None')
    print(f"[RC Monitor] Throttle={throttle_val}, Pitch={pitch_val}, Roll={roll_val}, Yaw={yaw_val}, Armed={vehicle.armed}")
    time.sleep(1)

print("Vehicle disarmed via RC. Closing connection.")
vehicle.close()
print("Done")
