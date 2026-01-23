#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地面站控制程序 (PC端)
发送控制指令到香橙派服务端
"""
import socket
import json
import time
import threading
from dronekit import connect, VehicleMode


class GroundStation:
    def __init__(self, server_ip='192.168.1.100', server_port=5000):
        self.server_ip = server_ip
        self.server_port = server_port
        self.socket = None
        self.connected = False
        self.command_history = []
        self.telemetry_data = {}
        self.lock = threading.Lock()

    def connect(self):
        """连接到香橙派服务端"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.server_ip, self.server_port))
            self.connected = True
            print(f"已连接到香橙派 {self.server_ip}:{self.server_port}")

            # 启动状态监听线程
            status_thread = threading.Thread(target=self._listen_status, daemon=True)
            status_thread.start()

            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def _listen_status(self):
        """监听香橙派返回的状态信息"""
        buffer = ""
        while self.connected:
            try:
                data = self.socket.recv(1024).decode('utf-8')
                if not data:
                    break

                buffer += data
                # 处理完整的JSON消息
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    try:
                        msg = json.loads(line)
                        with self.lock:
                            self.telemetry_data = msg
                        self._print_telemetry(msg)
                    except json.JSONDecodeError:
                        pass
            except Exception as e:
                if self.connected:
                    print(f"状态监听错误: {e}")
                break

    def _print_telemetry(self, data):
        """打印无人机状态信息"""
        print(f"[状态] 高度: {data.get('altitude', 'N/A'):.2f}m | "
              f"速度: {data.get('speed', 'N/A'):.2f}m/s | "
              f"电量: {data.get('battery', 'N/A')}% | "
              f"模式: {data.get('mode', 'N/A')}")

    def send_command(self, command_type, **params):
        """发送控制指令"""
        if not self.connected:
            print("未连接到香橙派")
            return False

        command = {
            'type': command_type,
            'params': params,
            'timestamp': time.time()
        }

        try:
            self.socket.send(json.dumps(command).encode('utf-8') + b'\n')
            self.command_history.append(command)
            print(f"发送指令: {command_type} - {params}")
            return True
        except Exception as e:
            print(f"发送失败: {e}")
            return False

    def arm_and_takeoff(self, altitude):
        """起飞到指定高度"""
        return self.send_command('arm_and_takeoff', altitude=altitude)

    def land(self):
        """降落"""
        return self.send_command('land')

    def set_mode(self, mode):
        """设置飞行模式"""
        return self.send_command('set_mode', mode=mode)

    def set_rc_override(self, roll=1500, pitch=1500, throttle=1500, yaw=1500, duration=None):
        """设置RC覆盖值"""
        params = {'roll': roll, 'pitch': pitch, 'throttle': throttle, 'yaw': yaw}
        if duration:
            params['duration'] = duration
        return self.send_command('set_rc_override', **params)

    def velocity_control(self, vx, vy, vz, duration=1):
        """速度控制"""
        return self.send_command('velocity', vx=vx, vy=vy, vz=vz, duration=duration)

    def yaw_control(self, direction, degrees, relative=True):
        """航向控制"""
        return self.send_command('yaw', direction=direction, degrees=degrees, relative=relative)

    def get_status(self):
        """获取当前无人机状态"""
        with self.lock:
            return self.telemetry_data.copy()

    def disconnect(self):
        """断开连接"""
        self.connected = False
        if self.socket:
            self.socket.close()
        print("已断开连接")


def interactive_control(gs):
    """交互式控制菜单"""
    print("\n=== 无人机地面站控制 ===")
    print("可用指令:")
    print("  1. 起飞 [高度] - arm_and_takeoff [altitude]")
    print("  2. 降落 - land")
    print("  3. 模式 [模式名] - set_mode [GUIDED/ALT_HOLD等]")
    print("  4. RC覆盖 [roll] [pitch] [throttle] [yaw] [持续时间]")
    print("  5. 速度控制 [vx] [vy] [vz] [持续时间]")
    print("  6. 航向控制 [方向:1/-1] [角度] [相对:1/0]")
    print("  7. 状态 - 查看当前无人机状态")
    print("  8. 退出 - 断开连接")
    print("======================\n")

    while True:
        try:
            cmd = input("控制指令> ").strip()
            if not cmd:
                continue

            parts = cmd.split()
            action = parts[0]

            if action in ['1', '起飞']:
                altitude = float(parts[1]) if len(parts) > 1 else 2.0
                gs.arm_and_takeoff(altitude)

            elif action in ['2', '降落']:
                gs.land()

            elif action in ['3', '模式']:
                mode = parts[1] if len(parts) > 1 else 'GUIDED'
                gs.set_mode(mode)

            elif action in ['4', 'RC覆盖']:
                roll = int(parts[1]) if len(parts) > 1 else 1500
                pitch = int(parts[2]) if len(parts) > 2 else 1500
                throttle = int(parts[3]) if len(parts) > 3 else 1500
                yaw = int(parts[4]) if len(parts) > 4 else 1500
                duration = float(parts[5]) if len(parts) > 5 else None
                gs.set_rc_override(roll, pitch, throttle, yaw, duration)

            elif action in ['5', '速度控制']:
                vx = float(parts[1]) if len(parts) > 1 else 0
                vy = float(parts[2]) if len(parts) > 2 else 0
                vz = float(parts[3]) if len(parts) > 3 else 0
                duration = float(parts[4]) if len(parts) > 4 else 1
                gs.velocity_control(vx, vy, vz, duration)

            elif action in ['6', '航向控制']:
                direction = int(parts[1]) if len(parts) > 1 else 1
                degrees = float(parts[2]) if len(parts) > 2 else 10
                relative = bool(int(parts[3])) if len(parts) > 3 else True
                gs.yaw_control(direction, degrees, relative)

            elif action in ['7', '状态']:
                status = gs.get_status()
                print(f"\n当前状态: {json.dumps(status, indent=2, ensure_ascii=False)}")

            elif action in ['8', '退出']:
                gs.disconnect()
                break

            else:
                print("未知指令，请重新输入")

        except KeyboardInterrupt:
            print("\n正在退出...")
            gs.disconnect()
            break
        except Exception as e:
            print(f"错误: {e}")


if __name__ == '__main__':
    # 配置香橙派的IP地址
    ORANGE_PI_IP = '192.168.1.100'  # 改为你的香橙派实际IP
    PORT = 5000

    gs = GroundStation(server_ip=ORANGE_PI_IP, server_port=PORT)

    if gs.connect():
        interactive_control(gs)
    else:
        print("无法连接到香橙派，请检查IP地址和端口")
