#!/bin/bash
# -*- coding: utf-8 -*-
"""
SITL启动脚本
启动ArduCopter SITL模拟器 + MAVProxy（带地图和控制台）
"""
set -e

ARDUPILOT_DIR="/home/undefined/Desktop/uav/ardupilot"
LOG_FILE="/tmp/sitl.log"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log() {
    echo -e "${GREEN}[SITL]${NC} $1"
}

error() {
    echo -e "${RED}[错误]${NC} $1"
}

stop_sitl() {
    echo -e "${YELLOW}停止SITL...${NC}"
    pkill -9 -f "sim_vehicle.py" 2>/dev/null || true
    pkill -9 -f "arducopter" 2>/dev/null || true
    pkill -9 -f "mavproxy.py" 2>/dev/null || true
    sleep 1
    echo "已停止"
}

start_sitl() {
    echo "========================================"
    echo "   启动 ArduCopter SITL 模拟器"
    echo "========================================"

    # 停止已存在的进程
    stop_sitl

    # 启动SITL
    log "启动SITL中..."
    cd "$ARDUPILOT_DIR"

    # 在后台启动SITL
    nohup ./build/sitl/bin/arducopter \
        --model + \
        --speedup 1 \
        --slave 0 \
        --defaults Tools/autotest/default_params/copter.parm \
        --sim-address=127.0.0.1 \
        -I0 \
        > "$LOG_FILE" 2>&1 &

    SITL_PID=$!
    log "SITL PID: $SITL_PID"

    # 等待SITL启动
    log "等待SITL启动..."
    for i in {1..20}; do
        sleep 1

        # 检查进程是否还在运行
        if ! kill -0 $SITL_PID 2>/dev/null; then
            error "SITL进程已退出"
            echo "日志:"
            tail -20 "$LOG_FILE"
            return 1
        fi

        # 检查端口是否开放
        if ss -tlnp 2>/dev/null | grep -q ":5760"; then
            log "SITL启动成功!"
            break
        fi

        if [ $i -eq 20 ]; then
            error "SITL启动超时"
            return 1
        fi
    done

    echo ""
    echo "========================================"
    echo "   SITL 已就绪"
    echo "========================================"
    echo ""
    echo "连接方式:"
    echo "  - MAVProxy: tcp:127.0.0.1:5760"
    echo "  - DroneKit: udp:127.0.0.1:14550"
    echo ""
    echo "启动MAVProxy（地图+控制台）:"
    echo "  mavproxy.py --master=tcp:127.0.0.1:5760 --map --console"
    echo ""
    echo "测试转向控制器:"
    echo "  cd /home/undefined/Desktop/uav/uav"
    echo "  python3 yaw_controller.py --auto"
    echo ""
    echo "按 Ctrl+C 停止SITL"
    echo "========================================"

    # 等待用户中断
    wait $SITL_PID
}

start_mavproxy() {
    log "启动MAVProxy..."
    /home/undefined/.local/bin/mavproxy.py --master=tcp:127.0.0.1:5760 --map --console
}

status_sitl() {
    echo "========================================"
    echo "   SITL 状态"
    echo "========================================"

    if ss -tlnp 2>/dev/null | grep -q ":5760"; then
        echo -e "${GREEN}✓ SITL正在运行${NC}"
        echo "端口:"
        ss -tlnp | grep -E ":5760|:1455" | sed 's/^/  /'
    else
        echo -e "${RED}✗ SITL未运行${NC}"
    fi

    echo ""
    echo "进程:"
    ps aux | grep -E "arducopter|mavproxy" | grep -v grep | sed 's/^/  /' || echo "  无"
}

if [ "$1" == "--stop" ]; then
    stop_sitl
elif [ "$1" == "--status" ]; then
    status_sitl
elif [ "$1" == "--mavproxy" ]; then
    start_mavproxy
else
    start_sitl
fi
