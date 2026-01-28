#!/bin/bash
#
# Ground-Air Cooperation System Launcher
# 设置PYTHONPATH并运行指定程序
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH}"

# 默认运行地面站
MODULE="ground_station.main"
SCRIPT="ground_station/main.py"

usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --pc       Run ground station (PC端)"
    echo "  --drone    Run drone server (无人机端)"
    echo "  --dog      Run dog server (机器狗端)"
    echo "  --sim      Run local simulation (本地模拟)"
    echo "  --help     Show this help"
    echo ""
    echo "Examples:"
    echo "  $0 --pc                    # 运行地面站"
    echo "  $0 --sim                  # 运行本地模拟"
    echo "  $0 --drone --fc-connection /dev/ttyACM0"
    echo ""
}

case "${1:-}" in
    --pc)
        MODULE="ground_station.main"
        SCRIPT="ground_station/main.py"
        ;;
    --drone)
        MODULE="drone.main"
        SCRIPT="drone/main.py"
        ;;
    --dog)
        MODULE="dog.main"
        SCRIPT="dog/main.py"
        ;;
    --sim|--simulation)
        MODULE=""
        SCRIPT="simulate.py"
        ;;
    --help|-h)
        usage
        exit 0
        ;;
    *)
        MODULE="ground_station.main"
        SCRIPT="ground_station/main.py"
        ;;
esac

shift

if [ -n "$MODULE" ]; then
    echo "Running: ${MODULE}"
    exec python3 -c "from ${MODULE} import main; main()" "$@"
else
    echo "Running: ${SCRIPT}"
    exec python3 "${SCRIPT}" "$@"
fi
