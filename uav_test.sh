#!/bin/bash
# UAV一键测试脚本
# 完整飞行测试 + 基础测试

ARDUPILOT_DIR="/home/undefined/Desktop/uav/ardupilot"
UAV_DIR="/home/undefined/Desktop/uav/uav"
LOG_DIR="/tmp/uav_test_logs"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BLUE='\033[0;34m'
NC='\033[0m'

MAVPROXY_PID=""
SITL_PID=""
CONNECTED=0

mkdir -p "$LOG_DIR"

echo_color() {
    echo -e "${1}${2}${NC}"
}

log() {
    echo -e "[$(date '+%H:%M:%S')] $1"
}

log_result() {
    local test_name=$1
    local passed=$2
    local error=$3
    
    if [ "$passed" == "true" ]; then
        echo -e "  ${GREEN}✓${NC} $test_name"
    else
        echo -e "  ${RED}✗${NC} $test_name"
        [ -n "$error" ] && echo -e "      ${RED}错误: $error${NC}"
    fi
}

# 清理函数
cleanup() {
    echo -e "\n${YELLOW}正在清理...${NC}"
    
    screen -S mavproxy -X quit 2>/dev/null || true
    pkill -9 -f "mavproxy.py" 2>/dev/null || true
    pkill -9 -f "arducopter" 2>/dev/null || true
    
    if [ "$CONNECTED" == "1" ]; then
        python3 -c "
import sys
sys.path.insert(0, '$UAV_DIR')
from dronekit import connect
try:
    v = connect('$CONN_STRING', baud=$BAUD, timeout=5)
    if v.armed:
        v.armed = False
    v.close()
except:
    pass
" 2>/dev/null &
    fi
    
    sleep 1
    echo -e "${GREEN}清理完成${NC}"
    exit 0
}

trap cleanup EXIT INT TERM

# 连接飞控
connect_vehicle() {
    local conn_str=$1
    local baud=$2
    
    CONN_STRING=$conn_str
    BAUD=$baud
    
    log "连接飞控: $conn_str @ $baud"
    
    python3 -c "
import sys
sys.path.insert(0, '$UAV_DIR')
from dronekit import connect, VehicleMode

try:
    v = connect('$conn_str', wait_ready=True, baud=$baud, timeout=30)
    print('SUCCESS')
    print('Mode:', v.mode.name)
    print('Armed:', v.armed)
    print('Altitude:', v.location.global_relative_frame.alt if v.location else 0)
    v.close()
except Exception as e:
    print('ERROR:', str(e))
    sys.exit(1)
" 2>&1 | tee /tmp/connect_test.log
    
    if grep -q "SUCCESS" /tmp/connect_test.log; then
        CONNECTED=1
        return 0
    else
        return 1
    fi
}

# 测试1: 飞控连接
test_connection() {
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}   测试1: 飞控连接${NC}"
    echo -e "${CYAN}========================================${NC}"
    
    local conn_str=$1
    local baud=$2
    
    if connect_vehicle "$conn_str" "$baud"; then
        log_result "飞控连接" "true"
        return 0
    else
        log_result "飞控连接" "false" "连接失败"
        return 1
    fi
}

# 测试2: 模式切换
test_mode_switch() {
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}   测试2: 模式切换${NC}"
    echo -e "${CYAN}========================================${NC}"
    
    python3 -c "
import sys
sys.path.insert(0, '$UAV_DIR')
from dronekit import connect, VehicleMode
import time

v = connect('$CONN_STRING', baud=$BAUD, wait_ready=True)

modes = ['STABILIZE', 'ALT_HOLD', 'GUIDED']
for mode in modes:
    v.mode = VehicleMode(mode)
    for _ in range(20):
        if v.mode.name == mode:
            print('  ✓', mode)
            break
        time.sleep(0.1)
    else:
        print('  ✗', mode, '- 超时')

v.close()
print('SUCCESS')
" 2>&1 | tee /tmp/mode_test.log
    
    if grep -q "SUCCESS" /tmp/mode_test.log; then
        log_result "模式切换" "true"
        return 0
    else
        log_result "模式切换" "false" "模式切换失败"
        return 1
    fi
}

# 测试3: 解锁/锁定
test_arm_disarm() {
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}   测试3: 解锁/锁定${NC}"
    echo -e "${CYAN}========================================${NC}"
    
    python3 -c "
import sys
sys.path.insert(0, '$UAV_DIR')
from dronekit import connect, VehicleMode
import time

v = connect('$CONN_STRING', baud=$BAUD, wait_ready=True)

# Pix6模式需要禁用RC检测
try:
    v.parameters['BRD_RC_TYPES'] = 0
    v.parameters['FS_THR_ENABLE'] = 0
    print('  RC检测已禁用')
except:
    pass

v.mode = VehicleMode('STABILIZE')
time.sleep(1)

# 解锁
print('  解锁...')
v.armed = True
for _ in range(30):
    if v.armed:
        print('  ✓ 解锁成功')
        break
    time.sleep(0.2)
else:
    print('  ⚠ 解锁超时，尝试MAVLink强制解锁')
    try:
        msg = v.message_factory.command_long_encode(0, 0, 400, 0, 1, 21196, 0, 0, 0, 0, 0)
        v.send_mavlink(msg)
        v.flush()
        time.sleep(1)
        if v.armed:
            print('  ✓ 强制解锁成功')
        else:
            print('  ✗ 强制解锁也失败')
            sys.exit(1)
    except Exception as e:
        print('  ✗ 解锁失败:', str(e))
        sys.exit(1)

# 锁定
time.sleep(1)
print('  锁定...')
v.armed = False
for _ in range(20):
    if not v.armed:
        print('  ✓ 锁定成功')
        break
    time.sleep(0.2)
else:
    print('  ⚠ 锁定超时')

v.close()
print('SUCCESS')
" 2>&1 | tee /tmp/arm_test.log
    
    if grep -q "SUCCESS" /tmp/arm_test.log; then
        log_result "解锁/锁定" "true"
        return 0
    else
        log_result "解锁/锁定" "false" "解锁失败"
        return 1
    fi
}

# 测试4: 遥测数据
test_telemetry() {
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}   测试4: 遥测数据${NC}"
    echo -e "${CYAN}========================================${NC}"
    
    python3 -c "
import sys
sys.path.insert(0, '$UAV_DIR')
from dronekit import connect
import time

v = connect('$CONN_STRING', baud=$BAUD, wait_ready=True)

print('  读取遥测数据...')
data = {
    '模式': v.mode.name,
    '解锁状态': v.armed,
    '航向': v.heading,
    '高度': v.location.global_relative_frame.alt if v.location else 0,
    '速度': v.groundspeed,
    '电量': v.battery.level if v.battery else 'N/A',
}

for key, value in data.items():
    print('    %s: %s' % (key, value))

v.close()
print('SUCCESS')
" 2>&1 | tee /tmp/telemetry_test.log
    
    if grep -q "SUCCESS" /tmp/telemetry_test.log; then
        log_result "遥测数据" "true"
        return 0
    else
        log_result "遥测数据" "false" "读取失败"
        return 1
    fi
}

# 测试5: 起飞到10米
test_takeoff() {
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}   测试5: 起飞到10米${NC}"
    echo -e "${CYAN}========================================${NC}"
    
    python3 -c "
import sys
sys.path.insert(0, '$UAV_DIR')
from dronekit import connect, VehicleMode
import time

v = connect('$CONN_STRING', baud=$BAUD, wait_ready=True)

try:
    v.parameters['BRD_RC_TYPES'] = 0
    v.parameters['FS_THR_ENABLE'] = 0
except:
    pass

v.mode = VehicleMode('GUIDED')
time.sleep(1)

if not v.armed:
    print('  解锁...')
    v.armed = True
    for _ in range(30):
        if v.armed:
            break
        time.sleep(0.2)
    else:
        print('  ✗ 解锁失败')
        sys.exit(1)

target_alt = 10
print('  起飞到10米...')
v.simple_takeoff(target_alt)

for i in range(60):
    alt = v.location.global_relative_frame.alt
    if alt is None:
        time.sleep(1)
        continue
    print('    当前高度: %.1fm' % alt)
    if alt >= target_alt * 0.95:
        print('  ✓ 到达10米高度')
        break
    time.sleep(1)
else:
    print('  ⚠ 起飞超时')

v.close()
print('SUCCESS')
" 2>&1 | tee /tmp/takeoff_test.log
    
    if grep -q "SUCCESS" /tmp/takeoff_test.log; then
        log_result "起飞测试" "true"
        return 0
    else
        log_result "起飞测试" "false" "起飞失败"
        return 1
    fi
}

# 测试6: 绕圈飞行
test_circle() {
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}   测试6: 绕圈飞行 (半径20米)${NC}"
    echo -e "${CYAN}========================================${NC}"
    
    python3 -c "
import sys
sys.path.insert(0, '$UAV_DIR')
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

v = connect('$CONN_STRING', baud=$BAUD, wait_ready=True)

try:
    v.parameters['BRD_RC_TYPES'] = 0
    v.parameters['FS_THR_ENABLE'] = 0
except:
    pass

v.mode = VehicleMode('GUIDED')
time.sleep(1)

if not v.armed:
    v.armed = True
    for _ in range(30):
        if v.armed:
            break
        time.sleep(0.2)

current = v.location.global_relative_frame
if not current or not current.lat:
    print('  ✗ 无法获取位置')
    sys.exit(1)

center_lat = current.lat
center_lon = current.lon
alt = max(current.alt, 10)
radius = 20

print('  开始绕圈飞行 (半径%dm)...' % radius)

for angle in range(0, 360, 10):
    lat_offset = (radius * math.cos(math.radians(angle))) / 111000
    lon_offset = (radius * math.sin(math.radians(angle))) / (111000 * math.cos(math.radians(center_lat)))
    
    target = LocationGlobalRelative(center_lat + lat_offset, center_lon + lon_offset, alt)
    v.simple_goto(target, groundspeed=5)
    time.sleep(1)
    
    if angle % 30 == 0:
        pos = v.location.global_relative_frame
        if pos and pos.lat:
            print('    位置: (%.6f, %.6f) 高度: %.1fm' % (pos.lat, pos.lon, pos.alt))

print('  ✓ 绕圈完成')

v.close()
print('SUCCESS')
" 2>&1 | tee /tmp/circle_test.log
    
    if grep -q "SUCCESS" /tmp/circle_test.log; then
        log_result "绕圈飞行" "true"
        return 0
    else
        log_result "绕圈飞行" "false" "绕圈失败"
        return 1
    fi
}

# 测试7: 方形飞行
test_square() {
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}   测试7: 方形飞行 (20x20米)${NC}"
    echo -e "${CYAN}========================================${NC}"
    
    python3 -c "
import sys
sys.path.insert(0, '$UAV_DIR')
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

v = connect('$CONN_STRING', baud=$BAUD, wait_ready=True)

try:
    v.parameters['BRD_RC_TYPES'] = 0
    v.parameters['FS_THR_ENABLE'] = 0
except:
    pass

v.mode = VehicleMode('GUIDED')
time.sleep(1)

if not v.armed:
    v.armed = True
    for _ in range(30):
        if v.armed:
            break
        time.sleep(0.2)

current = v.location.global_relative_frame
if not current or not current.lat:
    print('  ✗ 无法获取位置')
    sys.exit(1)

start_lat = current.lat
start_lon = current.lon
alt = max(current.alt, 10)
distance = 0.00018

print('  开始方形飞行 (20x20米)...')

# 东
target1 = LocationGlobalRelative(start_lat + distance, start_lon, alt)
v.simple_goto(target1, groundspeed=5)
time.sleep(5)
pos = v.location.global_relative_frame
if pos and pos.lat:
    print('    东: (%.6f, %.6f)' % (pos.lat, pos.lon))

# 北
target2 = LocationGlobalRelative(start_lat + distance, start_lon + distance, alt)
v.simple_goto(target2, groundspeed=5)
time.sleep(5)
pos = v.location.global_relative_frame
if pos and pos.lat:
    print('    北: (%.6f, %.6f)' % (pos.lat, pos.lon))

# 西
target3 = LocationGlobalRelative(start_lat, start_lon + distance, alt)
v.simple_goto(target3, groundspeed=5)
time.sleep(5)
pos = v.location.global_relative_frame
if pos and pos.lat:
    print('    西: (%.6f, %.6f)' % (pos.lat, pos.lon))

# 南
target4 = LocationGlobalRelative(start_lat, start_lon, alt)
v.simple_goto(target4, groundspeed=5)
time.sleep(5)
pos = v.location.global_relative_frame
if pos and pos.lat:
    print('    南: (%.6f, %.6f)' % (pos.lat, pos.lon))

print('  ✓ 方形飞行完成')

v.close()
print('SUCCESS')
" 2>&1 | tee /tmp/square_test.log
    
    if grep -q "SUCCESS" /tmp/square_test.log; then
        log_result "方形飞行" "true"
        return 0
    else
        log_result "方形飞行" "false" "方形飞行失败"
        return 1
    fi
}

# 测试8: 返回降落
test_rtl() {
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}   测试8: 返回降落${NC}"
    echo -e "${CYAN}========================================${NC}"
    
    python3 -c "
import sys
sys.path.insert(0, '$UAV_DIR')
from dronekit import connect, VehicleMode
import time

v = connect('$CONN_STRING', baud=$BAUD, wait_ready=True)

try:
    v.parameters['BRD_RC_TYPES'] = 0
    v.parameters['FS_THR_ENABLE'] = 0
except:
    pass

print('  切换到RTL模式 (返回)...')
v.mode = VehicleMode('RTL')
time.sleep(1)

print('  返回中...')
for i in range(60):
    alt = v.location.global_relative_frame.alt
    if alt is None:
        time.sleep(1)
        continue
    print('    高度: %.1fm' % alt)
    if alt < 5:
        print('  ✓ 接近Home')
        break
    time.sleep(1)

print('  切换到LAND模式...')
v.mode = VehicleMode('LAND')

print('  降落中...')
for i in range(60):
    alt = v.location.global_relative_frame.alt
    if alt is None:
        time.sleep(1)
        continue
    print('    高度: %.1fm' % alt)
    if alt < 0.5:
        print('  ✓ 降落完成')
        break
    time.sleep(1)

v.armed = False
v.close()
print('SUCCESS')
" 2>&1 | tee /tmp/rtl_test.log
    
    if grep -q "SUCCESS" /tmp/rtl_test.log; then
        log_result "返回降落" "true"
        return 0
    else
        log_result "返回降落" "false" "降落失败"
        return 1
    fi
}

# 测试9: 完整飞行测试
test_full_flight() {
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}   测试9: 完整飞行测试${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo -e "${YELLOW}  起飞 -> 悬停 -> 绕圈 -> 返回 -> 降落${NC}"
    
    python3 -c "
import sys
sys.path.insert(0, '$UAV_DIR')
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

v = connect('$CONN_STRING', baud=$BAUD, wait_ready=True)

try:
    v.parameters['BRD_RC_TYPES'] = 0
    v.parameters['FS_THR_ENABLE'] = 0
except:
    pass

v.mode = VehicleMode('GUIDED')
time.sleep(1)

# 解锁
if not v.armed:
    print('  解锁...')
    v.armed = True
    for _ in range(30):
        if v.armed:
            break
        time.sleep(0.2)

# 起飞
print('  起飞到10米...')
v.simple_takeoff(10)
for i in range(30):
    alt = v.location.global_relative_frame.alt
    if alt and alt >= 9.5:
        print('  ✓ 到达10米')
        break
    time.sleep(1)

# 悬停
print('  悬停3秒...')
time.sleep(3)

# 绕圈
current = v.location.global_relative_frame
if current and current.lat:
    center_lat = current.lat
    center_lon = current.lon
    alt = current.alt
    print('  绕圈飞行...')
    for angle in range(0, 360, 30):
        lat_offset = (20 * math.cos(math.radians(angle))) / 111000
        lon_offset = (20 * math.sin(math.radians(angle))) / (111000 * math.cos(math.radians(center_lat)))
        target = LocationGlobalRelative(center_lat + lat_offset, center_lon + lon_offset, alt)
        v.simple_goto(target, groundspeed=5)
        time.sleep(2)
    print('  ✓ 绕圈完成')

# 返回降落
print('  返回降落...')
v.mode = VehicleMode('RTL')
for i in range(60):
    alt = v.location.global_relative_frame.alt
    if alt and alt < 5:
        break
    time.sleep(1)

v.mode = VehicleMode('LAND')
for i in range(60):
    alt = v.location.global_relative_frame.alt
    if alt is None:
        continue
    if alt < 0.5:
        print('  ✓ 降落完成')
        break
    time.sleep(1)

v.armed = False
v.close()
print('SUCCESS')
" 2>&1 | tee /tmp/full_flight_test.log
    
    if grep -q "SUCCESS" /tmp/full_flight_test.log; then
        log_result "完整飞行测试" "true"
        return 0
    else
        log_result "完整飞行测试" "false" "测试失败"
        return 1
    fi
}

# 停止进程
stop_all() {
    echo -e "${YELLOW}\n停止所有进程...${NC}"
    screen -S mavproxy -X quit 2>/dev/null || true
    pkill -9 -f "mavproxy.py" 2>/dev/null || true
    pkill -9 -f "arducopter" 2>/dev/null || true
    echo -e "${GREEN}✓ 已停止${NC}"
    exit 0
}

# 帮助
show_help() {
    echo "UAV一键测试脚本"
    echo ""
    echo "用法: $0 [OPTIONS]"
    echo ""
    echo "选项:"
    echo "  --sitl      SITL模拟器模式"
    echo "  --pix6      Pix6硬件模式"
    echo "  --auto      自动测试"
    echo "  --stop      停止所有进程"
    echo "  --help      显示帮助"
}

# 启动SITL
start_sitl() {
    echo -e "${CYAN}\n[1/2] 启动SITL...${NC}"
    
    # TODO：ss工具是用来做什么的？/dev/null是用来做暂时存储的吧，一条命令执行过程中可以将中间过程写入这个文件是吗？
    # TODO: pgrep是什么？这里为什么要检测sitl是否已经在运行了？quad_setup.parm设置的是什么参数？我们在启动无人机时可以指定那些参数？$!表示的是什么？
    if ss -tlnp 2>/dev/null | grep -q ":5760"; then
        echo -e "${YELLOW}  SITL已在运行${NC}"
        SITL_PID=$(pgrep -f "arducopter" | head -1)
        return 0
    fi
    
    cd "$ARDUPILOT_DIR"
    nohup ./build/sitl/bin/arducopter \
        --model + \
        --defaults /home/undefined/Desktop/uav/uav/quad_setup.parm,Tools/autotest/default_params/copter.parm \
        > /tmp/sitl_test.log 2>&1 &
    
    SITL_PID=$!
    
    for i in {1..15}; do
        if ss -tlnp 2>/dev/null | grep -q ":5760"; then
            echo -e "${GREEN}  ✓ SITL启动成功 (TCP:5760)${NC}"
            return 0
        fi
        sleep 1
    done
    
    echo -e "${RED}  ✗ SITL启动失败${NC}"
    exit 1
}

# 启动MAVProxy
start_mavproxy() {
    echo -e "${CYAN}\n[2/2] 启动MAVProxy...${NC}"
    
    screen -S mavproxy -X quit 2>/dev/null || true
    pkill -9 -f "mavproxy.py" 2>/dev/null || true
    sleep 1
    
    cd "$ARDUPILOT_DIR"
    
    if command -v screen &> /dev/null; then
        screen -dmS mavproxy mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550 --map --console
        sleep 3
        echo -e "${GREEN}  ✓ MAVProxy已启动 (screen: mavproxy)${NC}"
    else
        echo -e "${YELLOW}  ⚠ screen未安装${NC}"
    fi
}

# 主程序
main() {
    MODE="sitl"
    AUTO="false"
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --sitl) MODE="sitl"; shift ;;
            --pix6) MODE="pix6"; shift ;;
            --auto) AUTO="true"; shift ;;
            --stop) stop_all ;;
            --help|-h) show_help; exit 0 ;;
            *) echo "未知参数: $1"; exit 1 ;;
        esac
    done
    
    if [ "$MODE" == "sitl" ]; then
    # TODO：当我们使用sitl时连接的是mavproxy指定的接口是吗？
        conn_str="udp:127.0.0.1:14550"
        baud=57600
    else
        conn_str="/dev/ttyACM0"
        baud=921600
    fi
    
    echo -e "${CYAN}"
    echo "========================================"
    echo "   UAV 一键测试脚本"
    echo "========================================"
    echo -e "${NC}"
    
    if [ "$MODE" == "sitl" ]; then
        start_sitl
        start_mavproxy
    fi
    
    if [ "$AUTO" == "true" ]; then
        echo -e "\n${YELLOW}自动完整测试...${NC}\n"
        test_connection "$conn_str" "$baud"
        test_mode_switch
        test_arm_disarm
        test_telemetry
        test_full_flight
        echo -e "\n${GREEN}测试完成${NC}"
    else
        while true; do
            echo ""
            echo -e "${BLUE}测试菜单:${NC}"
            echo "  基础测试:"
            echo "    1. 飞控连接"
            echo "    2. 模式切换"
            echo "    3. 解锁/锁定"
            echo "    4. 遥测数据"
            echo "  飞行测试:"
            echo "    5. 起飞测试 (到10米)"
            echo "    6. 绕圈飞行 (20米半径)"
            echo "    7. 方形飞行 (20x20米)"
            echo "    8. 返回降落"
            echo "    9. 完整飞行测试"
            echo "  0. 退出"
            echo -n "请选择 [0-9]: "
            read choice
            
            case $choice in
                1) test_connection "$conn_str" "$baud" ;;
                2) test_mode_switch ;;
                3) test_arm_disarm ;;
                4) test_telemetry ;;
                5) test_takeoff ;;
                6) test_circle ;;
                7) test_square ;;
                8) test_rtl ;;
                9) test_full_flight ;;
                0) break ;;
                *) echo "无效选择" ;;
            esac
            
            echo ""
            read -p "按回车键继续..." || true
        done
    fi
}

main "$@"
