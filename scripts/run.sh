#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
export PYTHONPATH="${PROJECT_ROOT}:${PROJECT_ROOT}/src:${PYTHONPATH}"

ENTRYPOINT="src/ground_station/ground_station.py"

usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --pc       Run ground station (PC端)"
    echo "  --drone    Run drone server (无人机端)"
    echo "  --dog      Run dog motion adapter (机器狗端适配器)"
    echo "  --sim      Run simulation (路径融合仿真)"
    echo "  --help     Show this help"
    echo ""
    echo "Examples:"
    echo "  $0 --pc                    # 运行地面站"
    echo "  $0 --sim                   # 运行仿真"
    echo "  $0 --drone --connection /dev/ttyACM0"
    echo ""
}

case "${1:-}" in
    --pc)
        ENTRYPOINT="src/ground_station/ground_station.py"
        shift
        ;;
    --drone)
        ENTRYPOINT="src/drone/drone_server.py"
        shift
        ;;
    --dog)
        ENTRYPOINT="src/dog/motion_adapter.py"
        shift
        ;;
    --sim|--simulation)
        ENTRYPOINT="simulation/main_path_fusion.py"
        shift
        ;;
    --help|-h)
        usage
        exit 0
        ;;
esac

echo "Running: ${ENTRYPOINT}"
exec python3 "${PROJECT_ROOT}/${ENTRYPOINT}" "$@"
