#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
香橙派服务端程序
接收PC端指令并控制无人机
"""
import socket
import json
import time
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import RPi.GPIO as GPIO


class DroneServer:
    def __init__(self, host='0.0.0.0', port=5000):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.running = False
        self.vehicle = None

        # 超声波引脚（如果使用）
        self.TRIG = 23
        self.ECHO = 24
        self.distance = None

        # 传感器和状态数据
        self.state = {
            'altitude': 0,
            'speed': 0,
            'battery': 100,
            'mode': 'UNKNOWN',
            'armed': False,
            'latitude': 0,
            'longitude': 0,
            'distance': None
        }
        self.state_lock = threading.Lock()

        # 摄像头和图像处理变量（如果使用）
        self.center1 = [None, None]
        self.radius1 = None

    def connect_vehicle(self, connection_string='/dev/ttyACM0', baud=921600):
        """连接飞控"""
        try:
            print(f"正在连接飞控: {connection_string}")
            self.vehicle = connect(connection_string, wait_ready=True, baud=baud)
            print("飞控连接成功!")
            return True
        except Exception as e:
            print(f"飞控连接失败: {e}")
            return False

    def init_ultrasonic(self):
        """初始化超声波传感器"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.TRIG, GPIO.OUT)
            GPIO.setup(self.ECHO, GPIO.IN)
            print("超声波传感器初始化完成")
        except Exception as e:
            print(f"超声波初始化失败: {e}")

    def measure_distance(self):
        """测量距离"""
        try:
            GPIO.output(self.TRIG, True)
            time.sleep(0.00001)
            GPIO.output(self.TRIG, False)

            pulse_start = time.time()
            while GPIO.input(self.ECHO) == 0:
                pulse_start = time.time()

            pulse_end = time.time()
            while GPIO.input(self.ECHO) == 1:
                pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            return round(distance, 2)
        except Exception:
            return None

    def start_server(self):
        """启动TCP服务器"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.running = True
        print(f"服务器启动，监听 {self.host}:{self.port}")

        # 启动状态广播线程
        broadcast_thread = threading.Thread(target=self._broadcast_status, daemon=True)
        broadcast_thread.start()

        # 启动传感器更新线程
        sensor_thread = threading.Thread(target=self._update_sensors, daemon=True)
        sensor_thread.start()

        self._accept_connection()

    def _accept_connection(self):
        """接受客户端连接"""
        while self.running:
            try:
                print("等待PC客户端连接...")
                self.client_socket, addr = self.server_socket.accept()
                print(f"PC客户端已连接: {addr}")
                self._handle_client()
            except Exception as e:
                if self.running:
                    print(f"连接错误: {e}")

    def _handle_client(self):
        """处理客户端指令"""
        buffer = ""
        while self.running:
            try:
                data = self.client_socket.recv(1024).decode('utf-8')
                if not data:
                    break

                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    try:
                        command = json.loads(line)
                        self._execute_command(command)
                    except json.JSONDecodeError:
                        print(f"无效的JSON指令: {line}")
            except Exception as e:
                print(f"客户端通信错误: {e}")
                break

        if self.client_socket:
            self.client_socket.close()
        print("PC客户端已断开")

    def _execute_command(self, command):
        """执行指令"""
        cmd_type = command.get('type')
        params = command.get('params', {})

        print(f"执行指令: {cmd_type} - {params}")

        try:
            if cmd_type == 'arm_and_takeoff':
                altitude = params.get('altitude', 2.0)
                self.arm_and_takeoff(altitude)

            elif cmd_type == 'land':
                self.land()

            elif cmd_type == 'set_mode':
                mode = params.get('mode', 'GUIDED')
                self.set_mode(mode)

            elif cmd_type == 'set_rc_override':
                self.set_rc_override(
                    roll=params.get('roll', 1500),
                    pitch=params.get('pitch', 1500),
                    throttle=params.get('throttle', 1500),
                    yaw=params.get('yaw', 1500),
                    duration=params.get('duration')
                )

            elif cmd_type == 'velocity':
                self.velocity_control(
                    vx=params.get('vx', 0),
                    vy=params.get('vy', 0),
                    vz=params.get('vz', 0),
                    duration=params.get('duration', 1)
                )

            elif cmd_type == 'yaw':
                self.yaw_control(
                    direction=params.get('direction', 1),
                    degrees=params.get('degrees', 10),
                    relative=params.get('relative', True)
                )

            elif cmd_type == 'emergency_stop':
                self.emergency_stop()

            else:
                print(f"未知指令类型: {cmd_type}")

        except Exception as e:
            print(f"指令执行失败: {e}")

    def _update_sensors(self):
        """更新传感器数据"""
        while self.running:
            try:
                if self.vehicle:
                    with self.state_lock:
                        self.state['altitude'] = self.vehicle.location.global_relative_frame.alt or 0
                        self.state['battery'] = self.vehicle.battery.level or 100
                        self.state['mode'] = self.vehicle.mode.name
                        self.state['armed'] = self.vehicle.armed

                        # 计算水平速度
                        vx = self.vehicle.velocity[0] if len(self.vehicle.velocity) > 0 else 0
                        vy = self.vehicle.velocity[1] if len(self.vehicle.velocity) > 1 else 0
                        self.state['speed'] = (vx**2 + vy**2)**0.5

                        self.state['latitude'] = self.vehicle.location.global_frame.lat or 0
                        self.state['longitude'] = self.vehicle.location.global_frame.lon or 0

                # 超声波测距
                self.distance = self.measure_distance()
                with self.state_lock:
                    self.state['distance'] = self.distance

            except Exception as e:
                print(f"传感器更新错误: {e}")

            time.sleep(0.1)

    def _broadcast_status(self):
        """向PC客户端广播状态"""
        while self.running:
            if self.client_socket:
                try:
                    with self.state_lock:
                        status = self.state.copy()

                    msg = json.dumps(status, ensure_ascii=False) + '\n'
                    self.client_socket.send(msg.encode('utf-8'))
                except Exception as e:
                    print(f"状态广播失败: {e}")
                    break

            time.sleep(0.5)

    # ===== 无人机控制函数 =====

    def arm_and_takeoff(self, altitude):
        """起飞到指定高度"""
        if not self.vehicle:
            print("飞控未连接")
            return False

        print(f"起飞到 {altitude} 米")

        while not self.vehicle.is_armable:
            print("等待飞控初始化...")
            time.sleep(1)

        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print("等待解锁...")
            time.sleep(1)

        self.vehicle.simple_takeoff(altitude)

        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt
            print(f"当前高度: {current_alt:.2f}m")
            if current_alt >= altitude * 0.95:
                print("到达目标高度")
                break
            time.sleep(0.5)

        return True

    def land(self):
        """降落"""
        if self.vehicle:
            print("开始降落")
            self.vehicle.mode = VehicleMode("LAND")
            return True
        return False

    def set_mode(self, mode):
        """设置飞行模式"""
        if self.vehicle:
            try:
                self.vehicle.mode = VehicleMode(mode)
                print(f"切换到 {mode} 模式")
                return True
            except Exception as e:
                print(f"模式切换失败: {e}")
        return False

    def set_rc_override(self, roll, pitch, throttle, yaw, duration=None):
        """RC覆盖控制"""
        if self.vehicle:
            start_time = time.time()
            while True:
                self.vehicle.channels.overrides = {
                    '1': roll,
                    '2': pitch,
                    '3': throttle,
                    '4': yaw
                }

                if duration and (time.time() - start_time) >= duration:
                    break
                if not duration:
                    break

                time.sleep(0.1)

            # 清除覆盖
            self.vehicle.channels.overrides = {}
            return True
        return False

    def velocity_control(self, vx, vy, vz, duration=1):
        """速度控制"""
        if self.vehicle:
            start_time = time.time()
            while time.time() - start_time < duration:
                msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                    0, 0, 0,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,
                    0, 0, 0,
                    vx, vy, vz,
                    0, 0, 0,
                    0, 0
                )
                self.vehicle.send_mavlink(msg)
                time.sleep(0.1)
            return True
        return False

    def yaw_control(self, direction, degrees, relative=True):
        """航向控制"""
        if self.vehicle:
            is_relative = 1 if relative else 0
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                degrees,
                0,
                direction,
                is_relative,
                0, 0, 0
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            print(f"航向调整: {direction} {degrees}度")
            return True
        return False

    def emergency_stop(self):
        """紧急停止"""
        if self.vehicle:
            print("紧急停止!")
            self.vehicle.channels.overrides = {}
            self.vehicle.mode = VehicleMode("LAND")
            return True
        return False

    def shutdown(self):
        """关闭服务器和连接"""
        print("正在关闭...")
        self.running = False

        if self.client_socket:
            self.client_socket.close()

        if self.server_socket:
            self.server_socket.close()

        if self.vehicle:
            try:
                self.vehicle.channels.overrides = {}
                self.vehicle.close()
            except:
                pass

        try:
            GPIO.cleanup()
        except:
            pass

        print("服务器已关闭")


def main():
    server = DroneServer(host='0.0.0.0', port=5000)

    # 连接飞控（根据实际情况修改连接字符串）
    # 本地连接: '/dev/ttyACM0' (USB) 或 '/dev/ttyS0' (串口)
    # 网络连接: '192.168.1.100:14552' (通过UDP连接)
    VEHICLE_CONNECTION = '/dev/ttyACM0'  # 改为实际连接端口
    BAUD_RATE = 921600

    if server.connect_vehicle(VEHICLE_CONNECTION, BAUD_RATE):
        # 如果使用超声波，取消下面注释
        # server.init_ultrasonic()

        try:
            server.start_server()
        except KeyboardInterrupt:
            print("\n接收到中断信号")
        finally:
            server.shutdown()
    else:
        print("飞控连接失败，服务器无法启动")


if __name__ == '__main__':
    main()
