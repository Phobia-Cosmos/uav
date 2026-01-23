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
from dronekit import connect, VehicleMode
from pymavlink import mavutil


class DroneServer:
    def __init__(self, host='0.0.0.0', port=5000):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.running = False
        self.vehicle = None

        # 状态数据
        self.state = {
            'altitude': 0,
            'speed': 0,
            'battery': 100,
            'mode': 'UNKNOWN',
            'armed': False,
            'latitude': 0,
            'longitude': 0,
            'pitch': 0,
            'roll': 0,
            'yaw': 0
        }
        self.state_lock = threading.Lock()

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

    def start_server(self):
        """启动TCP服务器"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.running = True
        print(f"服务器启动，监听 {self.host}:{self.port}")

        broadcast_thread = threading.Thread(target=self._broadcast_status, daemon=True)
        broadcast_thread.start()

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

            elif cmd_type == 'idle_test':
                duration = params.get('duration', 5)
                self.idle_test(duration)

            elif cmd_type == 'system_check':
                self.system_check()

            elif cmd_type == 'execute_rotation_maneuver':
                self.execute_rotation_maneuver(
                    altitude=params.get('altitude', 2),
                    angle=params.get('angle', 180),
                    direction=params.get('direction', 1),
                    speed=params.get('speed', 10)
                )

            elif cmd_type == 'get_attitude':
                self.print_attitude()

            elif cmd_type == 'test_motor':
                channel = params.get('channel', 3)
                pwm = params.get('pwm', 1100)
                duration = params.get('duration', 2)
                self.test_motor(channel, pwm, duration)

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
                        loc = self.vehicle.location.global_relative_frame
                        self.state['altitude'] = loc.alt if loc and loc.alt else 0

                        battery = self.vehicle.battery
                        if battery:
                            self.state['battery'] = battery.level if hasattr(battery, 'level') and battery.level else 100

                        self.state['mode'] = self.vehicle.mode.name if self.vehicle.mode else 'UNKNOWN'
                        self.state['armed'] = self.vehicle.armed

                        vel = self.vehicle.velocity
                        if vel and len(vel) >= 2:
                            self.state['speed'] = (vel[0]**2 + vel[1]**2)**0.5

                        frame = self.vehicle.location.global_frame
                        if frame:
                            self.state['latitude'] = frame.lat if frame.lat else 0
                            self.state['longitude'] = frame.lon if frame.lon else 0

                        att = self.vehicle.attitude
                        if att:
                            self.state['pitch'] = att.pitch
                            self.state['roll'] = att.roll
                            self.state['yaw'] = att.yaw

            except Exception as e:
                pass

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

    def print_attitude(self):
        """打印当前姿态"""
        if self.vehicle:
            att = self.vehicle.attitude
            if att:
                print(f"姿态: pitch={att.pitch:.2f}, roll={att.roll:.2f}, yaw={att.yaw:.2f}")
                return True
        return False

    def test_motor(self, channel=3, pwm=1100, duration=2):
        """
        测试单个电机

        Args:
            channel: 通道号 (1-8)
            pwm: PWM值 (1000-2000)
            duration: 测试时长（秒）
        """
        if not self.vehicle:
            print("错误: 飞控未连接")
            return False

        print(f"电机测试: 通道{channel}, PWM={pwm}, 持续{duration}秒")

        try:
            self.vehicle.mode = VehicleMode("STABILIZE")
            time.sleep(0.5)

            self.vehicle.armed = True
            time.sleep(2)

            if not self.vehicle.armed:
                print("解锁失败")
                return False

            print("电机启动...")
            self.vehicle.channels.overrides = {str(channel): pwm}

            time.sleep(duration)

            print("停止电机...")
            self.vehicle.channels.overrides = {}
            self.vehicle.armed = False

            time.sleep(0.5)
            self.vehicle.mode = VehicleMode("GUIDED")

            print("电机测试完成")
            return True

        except Exception as e:
            print(f"电机测试失败: {e}")
            try:
                self.vehicle.channels.overrides = {}
                self.vehicle.armed = False
            except:
                pass
            return False

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

    def idle_test(self, duration=5):
        """怠速测试（不起飞情况下测试电机响应）"""
        if not self.vehicle:
            print("错误: 飞控未连接")
            return False

        print(f"开始怠速测试，持续{duration}秒...")
        print("油门范围: 1050-1250 PWM")

        try:
            self.vehicle.mode = VehicleMode("STABILIZE")
            time.sleep(1)

            self.vehicle.armed = True
            time.sleep(2)

            if not self.vehicle.armed:
                print("解锁失败")
                return False

            throttle = 1050
            direction = 1
            throttle_step = 5
            min_throttle = 1050
            max_throttle = 1250

            start_time = time.time()

            print("电机怠速测试进行中...")

            while time.time() - start_time < duration:
                self.vehicle.channels.overrides = {'3': throttle}

                throttle += direction * throttle_step

                if throttle >= max_throttle:
                    throttle = max_throttle
                    direction = -1
                elif throttle <= min_throttle:
                    throttle = min_throttle
                    direction = 1

                elapsed = time.time() - start_time
                print(f"  [{elapsed:.1f}s] 油门: {throttle} PWM")

                time.sleep(0.1)

            print("怠速测试完成，停止电机...")
            self.vehicle.channels.overrides = {}
            self.vehicle.armed = False

            time.sleep(1)

            self.vehicle.mode = VehicleMode("GUIDED")

            print("怠速测试结束")
            return True

        except Exception as e:
            print(f"怠速测试失败: {e}")
            try:
                self.vehicle.channels.overrides = {}
                self.vehicle.armed = False
            except:
                pass
            return False

    def system_check(self):
        """系统自检"""
        print("开始系统自检...")

        results = {
            'passed': True,
            'checks': [],
            'timestamp': time.time()
        }

        check = {'name': '飞控连接', 'passed': False, 'message': ''}
        if self.vehicle:
            check['passed'] = True
            check['message'] = '已连接'
        else:
            check['message'] = '未连接'
            results['passed'] = False
        results['checks'].append(check)
        print(f"  [{'✓' if check['passed'] else '✗'}] {check['name']}: {check['message']}")

        if self.vehicle:
            check = {'name': '飞控可解锁', 'passed': False, 'message': ''}
            try:
                if self.vehicle.is_armable:
                    check['passed'] = True
                    check['message'] = '就绪'
                else:
                    check['message'] = '暂不可解锁'
            except Exception as e:
                check['message'] = f'错误: {e}'
                results['passed'] = False
            results['checks'].append(check)
            print(f"  [{'✓' if check['passed'] else '✗'}] {check['name']}: {check['message']}")

            check = {'name': '电池状态', 'passed': False, 'message': ''}
            try:
                battery = self.vehicle.battery
                voltage = getattr(battery, 'voltage', 0)
                level = getattr(battery, 'level', 0)
                if level:
                    check['passed'] = True
                check['message'] = f'{level}% ({voltage:.1f}V)'
            except Exception as e:
                check['message'] = f'错误: {e}'
                results['passed'] = False
            results['checks'].append(check)
            print(f"  [{'✓' if check['passed'] else '✗'}] {check['name']}: {check['message']}")

            check = {'name': '飞行模式', 'passed': False, 'message': ''}
            try:
                mode = self.vehicle.mode.name
                check['message'] = mode
                check['passed'] = True
            except Exception as e:
                check['message'] = f'错误: {e}'
                results['passed'] = False
            results['checks'].append(check)
            print(f"  [{'✓' if check['passed'] else '✗'}] {check['name']}: {check['message']}")

            check = {'name': '传感器姿态', 'passed': False, 'message': ''}
            try:
                att = self.vehicle.attitude
                if att:
                    pitch = att.pitch
                    roll = att.roll
                    yaw = att.yaw
                    if abs(pitch) < 1.5 and abs(roll) < 1.5:
                        check['passed'] = True
                    check['message'] = f'pitch:{pitch:.2f}, roll:{roll:.2f}, yaw:{yaw:.2f}'
                else:
                    check['message'] = '无姿态数据'
                    results['passed'] = False
            except Exception as e:
                check['message'] = f'错误: {e}'
                results['passed'] = False
            results['checks'].append(check)
            print(f"  [{'✓' if check['passed'] else '✗'}] {check['name']}: {check['message']}")

        passed_count = sum(1 for c in results['checks'] if c['passed'])
        total_count = len(results['checks'])

        print(f"\n系统自检完成: {passed_count}/{total_count} 项通过")

        if results['passed']:
            print("✓ 系统状态正常")
        else:
            print("✗ 系统存在问题，请检查")

        return results

    def execute_rotation_maneuver(self, altitude, angle, direction, speed=10):
        """执行完整旋转机动"""
        if not self.vehicle:
            print("错误: 飞控未连接")
            return False

        if angle <= 0:
            print("错误: angle必须大于0")
            return False

        if direction not in [1, -1]:
            print("错误: direction必须为1(顺时针)或-1(逆时针)")
            return False

        direction_name = "顺时针" if direction == 1 else "逆时针"

        print("=" * 50)
        print(f"开始完整旋转机动")
        print(f"  旋转高度: {altitude}米")
        print(f"  旋转角度: {angle}度 ({direction_name})")
        print(f"  旋转速度: {speed}度/秒")
        print("=" * 50)

        try:
            print(f"\n[Step 1] 起飞到 {altitude}米...")

            if not self.arm_and_takeoff(altitude):
                print("起飞失败")
                return False

            print(f"已到达目标高度 {altitude}米")

            print("\n[Step 2] 悬停2秒...")
            time.sleep(2)

            print(f"\n[Step 3] 执行旋转: {angle}度 ({direction_name})...")

            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                angle,
                speed,
                direction,
                1,
                0, 0, 0
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()

            wait_time = abs(angle) / speed + 2
            print(f"等待旋转完成 ({wait_time:.1f}秒)...")
            time.sleep(wait_time)

            print("\n[Step 4] 悬停3秒...")
            time.sleep(3)

            print("\n[Step 5] 开始降落...")
            self.land()

            print("\n" + "=" * 50)
            print("完整旋转机动执行完成")
            print("=" * 50)

            return True

        except Exception as e:
            print(f"旋转机动执行失败: {e}")
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

        print("服务器已关闭")


def main():
    import argparse

    parser = argparse.ArgumentParser(description='UAV地面站服务端')
    parser.add_argument('--pc-ip', default='192.168.1.50', help='PC IP地址')
    parser.add_argument('--connection', default='/dev/ttyACM0', help='飞控连接串')
    parser.add_argument('--baud', type=int, default=921600, help='波特率')
    parser.add_argument('--port', type=int, default=5000, help='服务端口')
    args = parser.parse_args()

    server = DroneServer(host='0.0.0.0', port=args.port)

    if server.connect_vehicle(args.connection, args.baud):
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
