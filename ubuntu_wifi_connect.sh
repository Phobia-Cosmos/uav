#!/bin/bash
#
# Ubuntu PC WiFi连接脚本
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

LOG_FILE="/tmp/wifi_connect.log"

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
    nmcli -t device status 2>/dev/null | grep "^wlp" | head -1 | cut -d: -f1
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

get_bssid() {
    local ssid="$1"
    nmcli -t -f BSSID,SSID device wifi 2>/dev/null | grep ":${ssid}$" | cut -d: -f1
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
        log "输入 3 扫描查看可用网络"
        return 1
    fi

    # 先断开现有连接
    local iface=$(get_wifi_interface)
    disconnect_all "$iface"

    if [ -z "$password" ]; then
        # 开放网络
        log "连接开放网络..."
        # 尝试用BSSID连接
        if [ -n "$bssid" ]; then
            sudo nmcli device wifi connect "$bssid" 2>&1
        else
            sudo nmcli device wifi connect -- "$ssid" 2>&1
        fi
    else
        # WPA/WPA2/WPA3网络
        log "连接加密网络..."
        # 优先用BSSID连接（解决特殊字符问题）
        if [ -n "$bssid" ]; then
            sudo nmcli device wifi connect "$bssid" password "$password" 2>&1
        else
            sudo nmcli device wifi connect -- "$ssid" password "$password" 2>&1
        fi
    fi

    sleep 3

    # 检查连接状态
    local connected_ssid=$(nmcli -t -f ACTIVE,SSID device wifi 2>/dev/null | grep "^yes" | cut -d: -f2)

    if [ "$connected_ssid" = "$ssid" ]; then
        log "连接成功!"
    else
        log "当前连接: $connected_ssid"
    fi

    sleep 2
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

show_menu() {
    echo ""
    echo "================================"
    echo "   Ubuntu WiFi连接工具"
    echo "================================"
    echo "可用网络:"
    echo "  1. $WIFI_1_NAME (无密码，可能需要认证)"
    echo "  2. $WIFI_2_NAME (个人热点)"
    echo "  3. 扫描并显示可用网络"
    echo "  4. 查看当前IP"
    echo "  5. 断开连接"
    echo "  6. 设置开机自启"
    echo "  7. 取消开机自启"
    echo "  0. 退出"
    echo "================================"
    echo -n "请选择 [0-7]: "
}

setup_autostart() {
    log "设置开机自启..."

    local script_path="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
    local service_file="/etc/systemd/system/wifi-connect-pc.service"

    cat > "$service_file" << EOF
[Unit]
Description=WiFi Auto Connect Service (PC)
After=network.target
Wants=network.target

[Service]
Type=oneshot
ExecStart=$script_path 1
RemainAfterExit=yes
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

    chmod 644 "$service_file"
    systemctl daemon-reload
    systemctl enable wifi-connect-pc.service

    log "已创建systemd服务: $service_file"
}

remove_autostart() {
    log "取消开机自启..."

    if [ -f "/etc/systemd/system/wifi-connect-pc.service" ]; then
        systemctl disable wifi-connect-pc.service
        rm -f /etc/systemd/system/wifi-connect-pc.service
        systemctl daemon-reload
        log "已移除systemd服务"
    fi
}

main() {
    check_root

    if ! command -v nmcli &> /dev/null; then
        echo "错误: nmcli未安装"
        exit 1
    fi

    log "========== WiFi连接工具启动 =========="

    WIFI_IFACE=$(get_wifi_interface)
    if [ -n "$WIFI_IFACE" ]; then
        log "检测到WiFi接口: $WIFI_IFACE"
    else
        log "未检测到WiFi接口"
    fi

    if [ -n "$1" ]; then
        case "$1" in
            1)
                connect_wifi_nmcli "$WIFI_1_NAME" "$WIFI_1_PASSWORD" "$WIFI_1_BSSID"
                ;;
            2)
                connect_wifi_nmcli "$WIFI_2_NAME" "$WIFI_2_PASSWORD" "$WIFI_2_BSSID"
                ;;
            --scan|-s)
                scan_networks
                ;;
            --status|-S)
                get_current_ip
                ;;
            --disconnect|-d)
                disconnect_all "$WIFI_IFACE"
                log "已断开连接"
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
                echo "用法: $0 [选项]"
                echo "选项:"
                echo "  1              连接i-HDU"
                echo "  2              连接Undefined"
                echo "  --scan, -s     扫描可用网络"
                echo "  --status, -S   查看当前IP"
                echo "  --disconnect   断开连接"
                echo "  --autostart    设置开机自启"
                echo "  --remove-autostart  取消开机自启"
                ;;
        esac
        exit 0
    fi

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
                scan_networks
                ;;
            4)
                get_current_ip
                ;;
            5)
                disconnect_all "$WIFI_IFACE"
                log "已断开连接"
                ;;
            6)
                setup_autostart
                ;;
            7)
                remove_autostart
                ;;
            0)
                echo "退出"
                exit 0
                ;;
            *)
                echo "无效选择"
                ;;
        esac

        echo ""
        echo -n "按回车键继续..."
        read -r
    done
}

main "$@"
