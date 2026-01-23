#!/bin/bash
# SITL Quick Start Script
set -e

ARDUPILOT_DIR="/home/undefined/Desktop/uav/ardupilot"
UAV_DIR="/home/undefined/Desktop/uav/uav"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo_color() {
    echo -e "${1}${2}${NC}"
}

stop_all() {
    echo -e "${YELLOW}\nStopping all processes...${NC}"
    pkill -9 -f "arducopter" 2>/dev/null || true
    pkill -9 -f "mavproxy.py" 2>/dev/null || true
    sleep 2
    echo "Done"
}

check_port() {
    local port=$1
    ss -tlnp 2>/dev/null | grep -q ":${port}" && return 0 || return 1
}

start_sitl() {
    echo -e "${CYAN}\n[1/3] Starting SITL (Quad mode)...${NC}"
    
    if check_port 5760; then
        echo -e "${YELLOW}SITL already running${NC}"
    else
        cd "$ARDUPILOT_DIR"
        
        nohup ./build/sitl/bin/arducopter \
            --model + \
            --speedup 1 \
            --defaults /home/undefined/Desktop/uav/uav/quad_setup.parm,Tools/autotest/default_params/copter.parm \
            --sim-address=127.0.0.1 \
            > /tmp/sitl.log 2>&1 &
        
        sleep 8
        
        if ! check_port 5760; then
            echo -e "${RED}SITL failed to start${NC}"
            tail -10 /tmp/sitl.log
            exit 1
        fi
        echo -e "${GREEN}SITL started successfully (TCP:5760)${NC}"
    fi
}

start_mavproxy() {
    echo -e "${CYAN}\n[2/3] Starting MAVProxy...${NC}"
    
    pkill -9 -f "mavproxy.py" 2>/dev/null || true
    sleep 1
    
    cd "$ARDUPILOT_DIR"
    
    if command -v screen &> /dev/null; then
        echo -e "${GREEN}Starting MAVProxy in screen session...${NC}"
        screen -dmS mavproxy mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550 --map --console
        sleep 3
        echo -e "${GREEN}MAVProxy started in screen session 'mavproxy'${NC}"
        echo -e "${YELLOW}Attach with: screen -r mavproxy${NC}"
    else
        echo -e "${GREEN}Starting MAVProxy...${NC}"
        mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550 --map --console
    fi
}

run_test() {
    echo -e "${CYAN}\n[3/3] Select test type...${NC}"
    
    cd "$UAV_DIR"
    
    while true; do
        echo ""
        echo "Test options:"
        echo "  1. Yaw controller test (rotate)"
        echo "  2. Quick circle test (15 sec)"
        echo "  3. Full flight test (takeoff -> circle -> hover -> return -> land)"
        echo "  4. Circle only (specify radius and altitude)"
        echo "  5. Takeoff + hover test"
        echo "  6. Arm check test"
        echo "  0. Exit"
        read -p "Select [0-6]: " choice
        
        case "$choice" in
            0)
                echo "Exiting..."
                break
                ;;
            1)
                echo -e "${GREEN}\nRunning yaw controller test...${NC}"
                python3 yaw_controller.py --connection udp:127.0.0.1:14550 --auto --angle 30
                ;;
            2)
                echo -e "${GREEN}\nRunning quick circle test...${NC}"
                python3 flight_test.py --mode quick --radius 20 --time 15
                ;;
            3)
                echo -e "${GREEN}\nRunning full flight test...${NC}"
                python3 flight_test.py --mode full --radius 20 --altitude 10
                ;;
            4)
                echo -e "${GREEN}\nRunning circle test...${NC}"
                read -p "Radius (m, default 20): " radius
                read -p "Altitude (m, default 10): " altitude
                python3 flight_test.py --mode circle --radius ${radius:-20} --altitude ${altitude:-10}
                ;;
            5)
                echo -e "${GREEN}\nRunning takeoff + hover test...${NC}"
                read -p "Hover time (sec, default 5): " time
                python3 flight_test.py --mode hover --time ${time:-5}
                ;;
            6)
                echo -e "${GREEN}\nRunning arm check test...${NC}"
                python3 flight_test.py --mode arm_test
                ;;
            *)
                echo "Invalid choice"
                ;;
        esac
        
        echo ""
        read -p "Press Enter to continue or 0 to exit..." continue_choice
        if [[ "$continue_choice" == "0" ]]; then
            break
        fi
    done
}

cleanup() {
    echo -e "${YELLOW}\nCleaning up...${NC}"
    pkill -9 -f "mavproxy.py" 2>/dev/null || true
    exit 0
}

trap cleanup INT TERM EXIT

if [[ "$1" == "--stop" ]]; then
    stop_all
elif [[ "$1" == "--sitl" ]]; then
    start_sitl
elif [[ "$1" == "--mavproxy" ]]; then
    start_mavproxy
elif [[ "$1" == "--test" ]]; then
    run_test
elif [[ "$1" == "--all" ]]; then
    stop_all
    start_sitl
    start_mavproxy
    run_test
elif [[ "$1" == "--help" ]] || [[ "$1" == "-h" ]]; then
    echo "SITL Quick Start Script"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --stop       Stop all SITL and MAVProxy processes"
    echo "  --sitl       Start SITL only"
    echo "  --mavproxy   Start MAVProxy only"
    echo "  --test       Run flight test (interactive menu)"
    echo "  --all        Start SITL, MAVProxy and run test"
    echo "  --help, -h   Show this help message"
    echo ""
    echo "Without arguments, interactive mode will be started."
else
    echo -e "${CYAN}\n========== SITL Quick Start ==========${NC}"
    
    if ! check_port 5760; then
        read -p "Start SITL? [Y/n]: " confirm
        if [[ "$confirm" =~ ^[Yy]$ ]] || [[ -z "$confirm" ]]; then
            stop_all
            start_sitl
            read -p "Start MAVProxy? [Y/n]: " mavconfirm
            if [[ "$mavconfirm" =~ ^[Yy]$ ]] || [[ -z "$mavconfirm" ]]; then
                start_mavproxy
            fi
        fi
    else
        read -p "Start MAVProxy? [Y/n]: " mavconfirm
        if [[ "$mavconfirm" =~ ^[Yy]$ ]] || [[ -z "$mavconfirm" ]]; then
            start_mavproxy
        fi
    fi
fi
