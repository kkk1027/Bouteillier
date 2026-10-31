#!/usr/bin/env bash
# 上下位机长连接：AP 模式切换脚本
#
# 【架构】上位机开热点，两块下位机板子都连它
#
#   上位机 Ubuntu ──── 开 AP (SSID: <WIFI_SSID>, IP: 172.20.10.5)
#      ├── ESP32 主板   连 AP → micro-ROS UDP 8888
#      ├── 雷达转接板   连 AP → TCP 8889
#      └── 手机/平板    连 AP → 浏览器打开 http://172.20.10.5:5000
#
# 【为什么 SSID 用 <WIFI_SSID>】
#   ESP32 固件里硬编码：
#       #define WIFI_SSID "<WIFI_SSID>"
#       #define MICROROS_AGENT_IP "172.20.10.5"
#   让上位机开一个同名热点、并用同一个 IP，**ESP32 固件一行都不用改**。
#
# 【重要：开 AP 会断网】
#   单网卡不能同时做 STA(连路由器) 和 AP(开热点)，
#   且两者必须同信道 —— 而 ESP8266/ESP32 只支持 2.4GHz，
#   所以不能在连 5GHz 路由器时开热点给它们用。
#   开 AP 期间：本机没外网，AI 走本地 Ollama（fishbot-qwen:3b）。
#   手机蜂窝数据仍可用。
#
# 用法：
#   ./ap_mode.sh on        # 切到 AP 模式（会断外网）
#   ./ap_mode.sh off       # 切回原来的 WiFi
#   ./ap_mode.sh status     # 看当前状态和已连设备
#   ./ap_mode.sh fix-ip     # 给两块板绑固定 IP（按 MAC）
#
set -euo pipefail

AP_CON=fishbot-ap
UPSTREAM="${UPSTREAM_CON:-小米智能57-101_5G}"
DEV=wlp0s20f3

need_sudo() {
    if ! sudo -n true 2>/dev/null; then
        echo "⚠️  需要 sudo 密码，请手动执行："
        echo "    sudo $*"
        exit 1
    fi
}

case "${1:-status}" in

on)
    echo "=== 切到 AP 模式（SSID: <WIFI_SSID>, 172.20.10.5）==="
    echo "⚠️  本机将失去外网访问；AI 可用本地 Ollama"
    sudo -n nmcli con up "$AP_CON" 2>/dev/null || {
        echo "需要密码，请手动执行： sudo nmcli con up $AP_CON"; exit 1; }
    sleep 3
    echo
    echo "--- 状态 ---"
    nmcli -t -f DEVICE,STATE,CONNECTION device | grep "$DEV" || true
    ip -4 addr show "$DEV" | grep -o "inet [0-9./]*" || true
    echo
    echo "--- 网页控制端 ---"
    IP=$(ip -4 addr show "$DEV" | grep -oP 'inet \K[0-9.]+' | head -1)
    echo "    本机:   http://127.0.0.1:5000"
    echo "    手机:   http://${IP:-172.20.10.5}:5000   （手机需连 <WIFI_SSID>）"
    echo
    echo "--- 等两块板接入（约 10~30 秒）---"
    echo "    用  ./ap_mode.sh status  查看"
    ;;

off)
    echo "=== 切回原来的 WiFi: $UPSTREAM ==="
    sudo -n nmcli con up "$UPSTREAM" 2>/dev/null || {
        echo "需要密码，请手动执行： sudo nmcli con up \"$UPSTREAM\""; exit 1; }
    sleep 3
    nmcli -t -f DEVICE,STATE,CONNECTION device | grep "$DEV" || true
    ip -4 addr show "$DEV" | grep -o "inet [0-9./]*" || true
    ;;

status)
    echo "=== 当前连接 ==="
    nmcli -t -f DEVICE,STATE,CONNECTION device | grep "$DEV" || true
    ip -4 addr show "$DEV" | grep -o "inet [0-9./]*" || true
    echo
    echo "=== AP 是否在广播 ==="
    if nmcli -t -f NAME connection show --active | grep -qx "$AP_CON"; then
        echo "  ✅ $AP_CON 已激活"
    else
        echo "  ⭕ $AP_CON 未激活（当前是普通 WiFi 模式）"
    fi
    echo
    echo "=== 已连接设备（AP 模式下）==="
    ip neigh show dev "$DEV" 2>/dev/null | grep -v FAILED || echo "  （无）"
    echo
    echo "=== 关键词检查 ==="
    echo "  micro_ros_agent 8888: $(ss -lunp 2>/dev/null | grep -c ':8888' || true) 个监听"
    echo "  tcp 桥接       8889: $(ss -ltnp 2>/dev/null | grep -c ':8889' || true) 个监听"
    ;;

fix-ip)
    # 按 MAC 给两块板绑定固定 IP，写在 dnsmasq 配置里
    # 需要先知道两块板的 MAC：AP 模式下用 ./ap_mode.sh status 看 ip neigh
    echo "=== 给下位机绑固定 IP ==="
    echo "先切到 AP 模式、让两块板连上，然后运行 ./ap_mode.sh status"
    echo "查到 MAC 后，执行（示例）："
    echo
    echo '  sudo tee /etc/NetworkManager/dnsmasq-shared.d/fishbot.conf <<EOF'
    echo '  dhcp-host=AA:BB:CC:DD:EE:01,172.20.10.10   # ESP32 主板'
    echo '  dhcp-host=AA:BB:CC:DD:EE:02,172.20.10.11   # 雷达转接板'
    echo '  EOF'
    echo '  sudo systemctl restart NetworkManager'
    echo
    echo "说明："
    echo "  板子侧仍从 DHCP 拿地址，只是每次都是同一个 —— 板子不用改任何配置。"
    echo "  注意 ESP32 固件里硬编码的 agent IP 是 172.20.10.5（=上位机自己），"
    echo "  所以**不要让绑定的 IP 和它冲突**。"
    ;;

*)
    echo "用法: $0 [on|off|status|fix-ip]"
    exit 1
    ;;
esac
