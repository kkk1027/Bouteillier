#!/usr/bin/env bash
# 实测 AX201 (iwlwifi) 能否 STA(2.4G) + AP 同信道并发
#
# 【要解决什么】
#   ESP32/ESP8266 只支持 2.4GHz。iwlwifi 的 STA+AP 并发**要求同信道**，
#   所以只要 STA 还连在 5GHz 路由器上，开出来的 AP 就是 5GHz，板子连不上。
#   如果能让 STA 改连路由器的 2.4G 频段（本项目实测该路由器有：
#   BSSID 24:CF:24:F9:96:65 / SSID 小米智能57-101 / ch1），
#   再把 AP 也设在 ch1，就可能同时上网 + 给板子开热点。
#
# 【本脚本做什么】
#   1. STA 切到 2.4G（小米智能57-101，密码已保存）
#   2. 验证外网可用
#   3. 尝试起 AP（fishbot-ap，ch1 同信道）
#   4. 跑连通性自检
#   5. **无论成败都自动切回原来的 5G**（保证 AI 助手能恢复通信）
#
# 【为什么必须自动回滚】
#   整个过程上位机会断网数分钟，AI 助手会失联。脚本把结论写进报告，
#   切回后直接看 /tmp/ap_2g4_test_report.txt 即可。
#
# 用法（需要 sudo）：
#   cd ~/bishe_ws/bishe-main/fishbot_ws
#   sudo -v                      # 先让 sudo 记住密码，避免中途卡住
#   ./tools/test_ap_2g4.sh
#
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPORT=/tmp/ap_2g4_test_report.txt
STA_2G4="小米智能57-101"
STA_5G="小米智能57-101_5G"
AP_CON="fishbot-ap"
DEV=wlp0s20f3

: > "$REPORT"
log() { echo "$@" | tee -a "$REPORT"; }
sec() { echo | tee -a "$REPORT"; echo "===== $* =====" | tee -a "$REPORT"; }

# 恢复原状：无论如何都要跑
restore() {
    sec "清理 / 回滚"
    log "  关闭 AP（如果起来了）..."
    nmcli con down "$AP_CON" >/dev/null 2>&1
    log "  切回 5G: $STA_5G"
    nmcli con up "$STA_5G" >/dev/null 2>&1
    sleep 5

    local ip
    ip=$(ip -4 addr show "$DEV" 2>/dev/null | grep -oP 'inet \K[0-9.]+' | head -1)
    log "  当前 IP: ${ip:-无}"
    if timeout 8 curl -s -o /dev/null -w '' "https://www.baidu.com" 2>/dev/null; then
        log "  ✅ 外网已恢复"
    else
        log "  ⚠️ 外网仍不通（可能需要手动：sudo nmcli con up \"$STA_5G\"）"
    fi
    sec "完成"
    log "报告：$REPORT"
}
trap restore EXIT INT TERM

log "AP 2.4G 并发实测   $(date '+%Y-%m-%d %H:%M:%S')"
log "网卡: $DEV  (iwlwifi, Intel AX201)"

# ---------------------------------------------------------------- 0
sec "0. 开始前状态"
nmcli -t -f DEVICE,STATE,CONNECTION device 2>/dev/null | grep "$DEV" | tee -a "$REPORT"
ip -4 addr show "$DEV" 2>/dev/null | grep -oP 'inet \K[0-9.]+/\d+' | tee -a "$REPORT"

# ---------------------------------------------------------------- 1
sec "1. STA 切到 2.4G（$STA_2G4 / ch1）"
# 让 2.4G 配置固定用 ch1，避免漫游到别的信道
nmcli con modify "$STA_2G4" 802-11-wireless.band bg >/dev/null 2>&1
nmcli con modify "$STA_2G4" 802-11-wireless.channel 1 >/dev/null 2>&1

if nmcli con up "$STA_2G4" >/dev/null 2>&1; then
    log "  ✅ 已连上 2.4G"
else
    log "  ❌ 连 2.4G 失败 —— 可能密码未保存或信号太弱，终止测试"
    exit 1
fi
sleep 6

log "--- 实际信道 ---"
CHAN=$(nmcli -f IN-USE,SSID,CHAN dev wifi list 2>/dev/null \
       | awk '$1=="*"' | awk '{print $3}')
log "  当前信道: ${CHAN:-未知}   期望: 1"
log "--- IP ---"
STA_IP=$(ip -4 addr show "$DEV" 2>/dev/null | grep -oP 'inet \K[0-9.]+' | head -1)
log "  ${STA_IP:-无}"

# ---------------------------------------------------------------- 2
sec "2. 验证外网（2.4G 上联是否可用）"
if timeout 10 curl -s -o /dev/null -w '' "https://www.baidu.com" 2>/dev/null; then
    log "  ✅ 外网可用"
    NET_OK=1
else
    log "  ❌ 外网不通 —— 后续即使 AP 起来也没意义，但仍继续测试并发能力"
    NET_OK=0
fi

# ---------------------------------------------------------------- 3
sec "3. 尝试起 AP（同信道 ch1）"
nmcli con modify "$AP_CON" 802-11-wireless.channel 1 >/dev/null 2>&1
AP_OUT=$(nmcli con up "$AP_CON" 2>&1)
AP_RC=$?
log "$AP_OUT" | sed 's/^/  /'
sleep 4

if ip -4 addr show "$DEV" 2>/dev/null | grep -q '172\.20\.10\.5'; then
    log "  ✅ AP 接口已起来（172.20.10.5 存在）"
    AP_OK=1
else
    log "  ❌ AP 未起来（未看到 172.20.10.5）"
    AP_OK=0
fi

log "--- 网卡模式 ---"
iwconfig "$DEV" 2>/dev/null | head -2 | tee -a "$REPORT"

# ---------------------------------------------------------------- 4
sec "4. 并发结论"
if [ "$AP_OK" = "1" ] && [ "$NET_OK" = "1" ]; then
    log "  🎉 成功：STA(2.4G) + AP(ch1) 可同时工作，且外网正常"
    log ""
    log "  → 可以把默认工作方式改成这个："
    log "      STA 常连 $STA_2G4 (ch1)"
    log "      AP  常开 $AP_CON (ch1)"
    log "      两块板连 <WIFI_SSID>，手机也连 <WIFI_SSID> 看网页"
elif [ "$AP_OK" = "1" ] && [ "$NET_OK" = "0" ]; then
    log "  ⚠️ AP 起来了，但 2.4G 上联没有外网"
    log "     说明并发生效，只是这个 2.4G 网络本身不通外网"
elif [ "$AP_OK" = "0" ]; then
    log "  ❌ AP 起不来 —— iwlwifi 在这个内核/固件下不支持 STA+AP 并发"
    log "     结论：必须用 USB WiFi 网卡或 USB 有线网卡"
fi

# ---------------------------------------------------------------- 5
sec "5. 若要验证板子连通，趁现在（AP 还开着）"
log "  另开终端执行："
log "    $WS/tools/verify_link.sh"
log ""
log "  本脚本会在退出时自动关 AP 并切回 5G。"

exit 0
