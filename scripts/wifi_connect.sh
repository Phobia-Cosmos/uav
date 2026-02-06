#!/bin/bash
#
# 香橙派WiFi连接脚本
# 使用nmcli管理WiFi连接
#

set -e

# WiFi配置
WIFI_1_NAME="i-HDU"
WIFI_1_PASSWORD=""
WIFI_1_BSSID="28:41:EC:29:30:10"  # 信号最强的i-HDU接入点

WIFI_2_NAME="Undefined"
WIFI_2_PASSWORD="lzh200341.."
WIFI_2_BSSID="36:C0:AA:ED:FC:62"

WIFI_3_NAME="2楼"
WIFI_3_PASSWORD="q1w2e3r4t5."
WIFI_3_BSSID=""

# PC IP地址（状态回传）
PC_IP="192.168.1.50"
PC_PORT="5001"

LOG_FILE="/var/log/wifi_connect.log"

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

check_root() {
    if [ "$EUID" -ne 0 ]; then
        echo "请使用root权限运行: sudo $0"
        exit 1
    fi
}

get_wifi_interface() {
    nmcli -t device status 2>/dev/null | grep "^wlan" | head -1 | cut -d: -f1
}

disconnect_all() {
    local iface=$1
    if [ -n "$iface" ]; then
        log "断开WiFi连接..."
        nmcli device disconnect "$iface" 2>/dev/null || true
        sleep 1
    fi
}

network_exists() {
    local ssid="$1"
    nmcli -t -f SSID device wifi 2>/dev/null | grep -Fxq "$ssid"
}

connect_wifi_nmcli() {
    local ssid="$1"
    local password="$2"
    local bssid="$3"

    log "正在连接WiFi: $ssid"

    # 检查网络是否存在
    if ! network_exists "$ssid"; then
        log "错误: 未找到网络 '$ssid'"
        log "请确保WiFi已开启，然后在信号范围内"
        log "输入 4 扫描查看可用网络"
        return 1
    fi

    # 先断开现有连接
    local iface=$(get_wifi_interface)
    disconnect_all "$iface"

    if [ -z "$password" ]; then
        # 开放网络
        log "连接开放网络..."
        if [ -n "$bssid" ]; then
            sudo nmcli device wifi connect "$bssid" 2>&1
        else
            sudo nmcli device wifi connect "$ssid" 2>&1
        fi
    else
        # WPA/WPA2/WPA3网络
        log "连接加密网络..."
        if [ -n "$bssid" ]; then
            sudo nmcli device wifi connect "$bssid" password "$password" 2>&1
        else
            sudo nmcli device wifi connect "$ssid" password "$password" 2>&1
        fi
    fi

    sleep 3

    # 检查连接状态
    local connected_ssid=$(nmcli -t -f ACTIVE,SSID device wifi 2>/dev/null | grep "^yes" | cut -d: -f2)

    if [ "$connected_ssid" = "$ssid" ]; then
        log "连接成功!"
        return 0
    else
        log "连接失败，当前连接: $connected_ssid"
        return 1
    fi

    # 获取IP地址
    sleep 2
    local ip=$(ip addr show "$iface" 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1 | head -1)

    if [ -n "$ip" ]; then
        log "IP地址: $ip"

        # 发送状态到PC
        send_status_to_pc "$ssid" "$ip"

        # 测试网络连通性
        if ping -c 1 -W 2 "8.8.8.8" &> /dev/null; then
            log "外网连接正常"

            if [ "$ssid" = "$WIFI_1_NAME" ]; then
                log "检测到i-HDU网络，可能需要网页认证"
                log "认证地址: http://login.hdu.edu.cn"
                log "账号: 251050092"
            fi
        else
            log "警告: 无法访问外网（可能需要认证或无网络）"
        fi
    fi

    echo "$ip"
    return 0
}

send_status_to_pc() {
    local ssid="$1"
    local ip="$2"
    local message="{\"type\": \"wifi_status\", \"ssid\": \"$ssid\", \"ip\": \"$ip\", \"timestamp\": $(date +%s)}"

    log "发送WiFi状态到PC: $message"

    # 尝试发送状态（UDP广播）
    echo "$message" | nc -u -w1 "$PC_IP" "$PC_PORT" 2>/dev/null || \
    echo "$message" | timeout 1 bash -c "cat > /dev/udp/$PC_IP/$PC_PORT" 2>/dev/null || \
    log "警告: 无法发送状态到PC（端口可能未开放）"
}

setup_autostart() {
    local script_path="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
    local service_file="/etc/systemd/system/wifi-connect.service"

    log "设置开机自动连接WiFi..."

    # 创建systemd服务（自动连接模式）
    cat > "$service_file" << EOF
[Unit]
Description=WiFi Auto Connect Service
After=network.target
Wants=network.target

[Service]
Type=oneshot
ExecStart=$script_path --auto
RemainAfterExit=yes
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

    chmod 644 "$service_file"
    systemctl daemon-reload
    systemctl enable wifi-connect.service

    log "已创建systemd服务: $service_file"
    log "系统将在启动时自动尝试连接WiFi"
}

remove_autostart() {
    log "取消开机自启..."

    if [ -f "/etc/systemd/system/wifi-connect.service" ]; then
        systemctl disable wifi-connect.service
        rm -f /etc/systemd/system/wifi-connect.service
        systemctl daemon-reload
        log "已移除systemd服务"
    fi
}

scan_networks() {
    echo ""
    echo "扫描可用WiFi网络..."
    echo ""
    nmcli device wifi list 2>/dev/null || echo "扫描失败"
}

get_current_ip() {
    local iface=$(get_wifi_interface)
    local ip=$(ip addr show "$iface" 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1 | head -1)

    if [ -z "$ip" ]; then
        echo "当前未连接网络"
    else
        local ssid=$(nmcli -t -f ACTIVE,SSID device wifi 2>/dev/null | grep "^yes" | cut -d: -f2)
        if [ -z "$ssid" ]; then
            ssid="未知"
        fi

        echo ""
        echo "================================"
        echo "   WiFi: $ssid"
        echo "   IP:   $ip"
        echo "================================"
    fi
}

auto_connect() {
    log "自动尝试连接可用WiFi..."
    log "按 Ctrl+C 中断"
    
    local wifi_list=(
        "$WIFI_1_NAME:$WIFI_1_PASSWORD:$WIFI_1_BSSID"
        "$WIFI_2_NAME:$WIFI_2_PASSWORD:$WIFI_2_BSSID"
        "$WIFI_3_NAME:$WIFI_3_PASSWORD:$WIFI_3_BSSID"
    )
    
    for wifi_info in "${wifi_list[@]}"; do
        IFS=':' read -r ssid password bssid <<< "$wifi_info"
        
        log "尝试连接: $ssid"
        
        if connect_wifi_nmcli "$ssid" "$password" "$bssid"; then
            log "自动连接成功!"
            return 0
        fi
        
        log "$ssid 连接失败，尝试下一个..."
        sleep 2
    done
    
    log "所有WiFi连接失败"
    return 1
}

show_menu() {
    echo ""
    echo "================================"
    echo "   香橙派 WiFi连接工具"
    echo "================================"
    echo "可用网络:"
    echo "  1. $WIFI_1_NAME (无密码，可能需要认证)"
    echo "  2. $WIFI_2_NAME (个人热点)"
    echo "  3. $WIFI_3_NAME"
    echo "  4. 扫描并显示可用网络"
    echo "  5. 查看当前IP"
    echo "  6. 断开连接"
    echo "  7. 自动连接（依次尝试所有WiFi）"
    echo "  8. 设置开机自动连接"
    echo "  9. 取消开机自动连接"
    echo "  0. 退出"
    echo "================================"
    echo -n "请选择 [0-9]: "
}

auto_connect() {
    log "自动尝试连接可用WiFi..."
    log "按 Ctrl+C 中断"
    
    local wifi_list=(
        "$WIFI_1_NAME:$WIFI_1_PASSWORD:$WIFI_1_BSSID"
        "$WIFI_2_NAME:$WIFI_2_PASSWORD:$WIFI_2_BSSID"
        "$WIFI_3_NAME:$WIFI_3_PASSWORD:$WIFI_3_BSSID"
    )
    
    for wifi_info in "${wifi_list[@]}"; do
        IFS=':' read -r ssid password bssid <<< "$wifi_info"
        
        log "尝试连接: $ssid"
        
        if connect_wifi_nmcli "$ssid" "$password" "$bssid"; then
            log "自动连接成功!"
            return 0
        fi
        
        log "$ssid 连接失败，尝试下一个..."
        sleep 2
    done
    
    log "所有WiFi连接失败"
    return 1
}

main() {
    check_root

    # 检查nmcli是否可用
    if ! command -v nmcli &> /dev/null; then
        echo "错误: nmcli未安装，请安装 network-manager"
        exit 1
    fi

    log "========== 香橙派WiFi连接工具启动 =========="

    WIFI_IFACE=$(get_wifi_interface)
    if [ -n "$WIFI_IFACE" ]; then
        log "检测到WiFi接口: $WIFI_IFACE"
    else
        log "未检测到WiFi接口"
    fi

    # 处理参数
    case "${1:-}" in
        --scan|-s)
            scan_networks
            exit 0
            ;;
        --status|-S)
            get_current_ip
            exit 0
            ;;
        --disconnect|-d)
            disconnect_all "$WIFI_IFACE"
            log "已断开连接"
            exit 0
            ;;
        --auto)
            auto_connect
            exit $?
            ;;
        --autostart)
            setup_autostart
            exit 0
            ;;
        --remove-autostart)
            remove_autostart
            exit 0
            ;;
        *)
            # 旧参数兼容
            if [ -n "$1" ]; then
                case "$1" in
                    1) connect_wifi_nmcli "$WIFI_1_NAME" "$WIFI_1_PASSWORD" "$WIFI_1_BSSID" ;;
                    2) connect_wifi_nmcli "$WIFI_2_NAME" "$WIFI_2_PASSWORD" "$WIFI_2_BSSID" ;;
                    3) connect_wifi_nmcli "$WIFI_3_NAME" "$WIFI_3_PASSWORD" "$WIFI_3_BSSID" ;;
                    *) echo "未知参数: $1" ;;
                esac
                exit $?
            fi
            ;;
    esac

    # 交互模式
    while true; do
        show_menu
        read -r choice

        case "$choice" in
            1)
                connect_wifi_nmcli "$WIFI_1_NAME" "$WIFI_1_PASSWORD" "$WIFI_1_BSSID"
                ;;
            2)
                connect_wifi_nmcli "$WIFI_2_NAME" "$WIFI_2_PASSWORD" "$WIFI_2_BSSID"
                ;;
            3)
                connect_wifi_nmcli "$WIFI_3_NAME" "$WIFI_3_PASSWORD" "$WIFI_3_BSSID"
                ;;
            4)
                scan_networks
                ;;
            5)
                get_current_ip
                ;;
            6)
                disconnect_all "$WIFI_IFACE"
                log "已断开连接"
                ;;
            7)
                auto_connect
                ;;
            8)
                setup_autostart
                ;;
            9)
                remove_autostart
                ;;
            0)
                echo "退出"
                exit 0
                ;;
            *)
                echo "无效选择，请重新输入"
                ;;
        esac

        echo ""
        echo -n "按回车键继续..."
        read -r
    done
}

main "$@"
