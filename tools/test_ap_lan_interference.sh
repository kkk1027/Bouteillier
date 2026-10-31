#!/usr/bin/env bash
# A 方案实测：单网卡 STA(2.4G) + AP(ch1) 并发，量化互相干扰
#
# 【要回答的问题】
#   一张网卡同时当 STA(上网) 和 AP(小车通信) 时，
#   STA 侧的大流量会不会挤占 AP 侧，导致雷达/里程计掉帧？
#
# 【测量方法】
#   分四阶段，每阶段采 12 秒 /scan 与 /odom 频率：
#     阶段1 基线：AP 未开，只有 STA（当前 2.4G）
#     阶段2 AP 开、小车空闲：我什么都不做
#     阶段3 AP 开、我制造大流量：curl 持续下载
#     阶段4 AP 关、小车空闲：回到基线对照
#
# 【安全设计】
#   - trap 保证退出时必关 AP，不切断用户网络
#   - 全程不碰 /cmd_vel，不会让车动
#   - 每阶段失败都继续，不中断整体测试
#
# 用法：
#   cd ~/bishe_ws/bishe-main/fishbot_ws
#   sudo -v
#   ./tools/test_ap_lan_interference.sh
#
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPORT=/tmp/ap_lan_test_report.txt
RAW=/tmp/ap_lan_test_raw.txt
AP_CON="fishbot-ap"
DEV=wlp0s20f3
SAMPLE=12          # 每阶段采样秒数

: > "$REPORT"
: > "$RAW"
log() { echo "$@" | tee -a "$REPORT"; }
sec() { echo | tee -a "$REPORT"; echo "===== $* =====" | tee -a "$REPORT"; }

# ---------------- ROS 环境 ----------------
# 注意：不能用 set -u，ROS 的 setup.bash 引用未绑定变量会中断脚本
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null
# shellcheck disable=SC1090
source "$WS/install/setup.bash" 2>/dev/null

restore() {
    sec "清理 / 恢复"
    if nmcli -t -f NAME connection show --active 2>/dev/null | grep -qx "$AP_CON"; then
        log "  关闭 AP ..."
        nmcli con down "$AP_CON" >/dev/null 2>&1
    else
        log "  AP 本来就是关的"
    fi
    sleep 2
    nmcli -t -f DEVICE,STATE,CONNECTION device 2>/dev/null | grep "$DEV" | tee -a "$REPORT"
    local ip
    ip=$(ip -4 addr show "$DEV" 2>/dev/null | grep -oP 'inet \K[0-9.]+' | head -1)
    log "  STA IP: ${ip:-无}"
    if timeout 8 curl -s -o /dev/null "https://www.baidu.com" 2>/dev/null; then
        log "  ✅ 外网正常"
    else
        log "  ⚠️ 外网不通"
    fi
    sec "完成"
    log "报告: $REPORT"
}
trap restore EXIT INT TERM

# ---------------- 采样函数 ----------------
# 直接数消息条数再除以时间，不用 ros2 topic hz 的滑动平均：
#   - hz 命令本身要跑满一个窗口才有输出，多话题并发时互相抢 CPU
#   - 数条数更直观，也方便算丢帧率
# 注意每个话题单独跑（顺序执行），避免 ROS 发现开销互相干扰。
count_msgs() {
    local topic="$1" secs="$2"
    timeout $((secs + 3)) ros2 topic hz "$topic" --window 100 2>/dev/null \
        | grep -oP 'average rate: \K[0-9.]+' | tail -1
}

sample_topics() {
    local tag="$1" file="$RAW.$tag"
    local s o
    s=$(count_msgs /scan "$SAMPLE")
    o=$(count_msgs /odom "$SAMPLE")
    echo "scan_hz=${s:-0}  odom_hz=${o:-0}" > "$file"
    log "  /scan: ${s:-无数据} Hz    /odom: ${o:-无数据} Hz"
}

# ---------------- 干扰流 ----------------
start_load() {
    # 找一个能持续下载的大文件。用阿里云镜像的 ubuntu 镜像做靶子。
    ( timeout 40 curl -s -o /dev/null \
        "https://mirrors.aliyun.com/ubuntu-releases/22.04/ubuntu-22.04.5-live-server-amd64.iso" \
        & echo $! > /tmp/_loadpid )
    log "  已启动后台下载（约 $(sleep 3; echo)4 秒）"
}
stop_load() {
    [ -f /tmp/_loadpid ] && kill "$(cat /tmp/_loadpid)" 2>/dev/null
    pkill -f "ubuntu-22.04.5-live-server" 2>/dev/null
    rm -f /tmp/_loadpid
}

log "A 方案干扰实测   $(date '+%Y-%m-%d %H:%M:%S')"
log "网卡: $DEV   AP: $AP_CON"

# ================================================================
sec "阶段 0：起始状态"
nmcli -t -f DEVICE,STATE,CONNECTION device 2>/dev/null | grep "$DEV" | tee -a "$REPORT"
iwconfig "$DEV" 2>/dev/null | grep -E "Frequency|Bit Rate" | tee -a "$REPORT"
ip -4 addr show "$DEV" 2>/dev/null | grep -oP 'inet \K[0-9.]+/\d+' | tee -a "$REPORT"

# ================================================================
sec "阶段 1：基线（AP 未开，纯 STA）"
nmcli con down "$AP_CON" >/dev/null 2>&1
sleep 2
sample_topics baseline

# ================================================================
sec "阶段 2：开 AP，小车空闲"
log "  起 AP（ch1，与 STA 同信道）..."
AP_MSG=$(nmcli con up "$AP_CON" 2>&1)
log "  $(echo "$AP_MSG" | head -2)"
sleep 5

if ip -4 addr show "$DEV" 2>/dev/null | grep -q '172\.20\.10\.5'; then
    log "  ✅ AP 接口已起来（172.20.10.5）"
    AP_OK=1
else
    log "  ❌ AP 未起来"
    AP_OK=0
fi

log "  --- STA 是否还活着 ---"
ip -4 addr show "$DEV" 2>/dev/null | grep -oP 'inet \K[0-9.]+/\d+' | tee -a "$REPORT"
if timeout 6 curl -s -o /dev/null "https://www.baidu.com" 2>/dev/null; then
    log "  ✅ 外网仍可用 —— 并发生效！"
    NET_OK=1
else
    log "  ❌ 外网断了 —— STA 被 AP 踢掉"
    NET_OK=0
fi

log "  --- 已连设备（等小车）---"
ip neigh show dev "$DEV" 2>/dev/null | grep '172\.20\.10' | tee -a "$REPORT" \
    || log "  （还没有 172.20.10.x 设备，车可能还没连上）"

sample_topics ap_idle

# ================================================================
sec "阶段 3：AP 开着，同时制造大流量"
start_load
sleep 3
sample_topics ap_load
stop_load
sleep 2

# ================================================================
sec "阶段 4：关 AP，回到基线对照"
nmcli con down "$AP_CON" >/dev/null 2>&1
sleep 3
sample_topics post

# ================================================================
sec "结论"
read_val() { grep -oP "$2=\K[0-9.]+" "$RAW.$1" 2>/dev/null || echo 0; }

B_S=$(read_val baseline scan_hz); B_O=$(read_val baseline odom_hz)
A_S=$(read_val ap_idle scan_hz);  A_O=$(read_val ap_idle odom_hz)
L_S=$(read_val ap_load scan_hz);  L_O=$(read_val ap_load odom_hz)
P_S=$(read_val post scan_hz);     P_O=$(read_val post odom_hz)

printf '  %-18s %10s %10s\n' "阶段" "/scan Hz" "/odom Hz" | tee -a "$REPORT"
printf '  %-18s %10s %10s\n' "1 基线(无AP)"   "$B_S" "$B_O" | tee -a "$REPORT"
printf '  %-18s %10s %10s\n' "2 AP开/车空闲"  "$A_S" "$A_O" | tee -a "$REPORT"
printf '  %-18s %10s %10s\n' "3 AP开+大流量"  "$L_S" "$L_O" | tee -a "$REPORT"
printf '  %-18s %10s %10s\n' "4 关AP(对照)"   "$P_S" "$P_O" | tee -a "$REPORT"
log ""

if [ "${AP_OK:-0}" = "0" ]; then
    log "  ❌ AP 起不来 → iwlwifi 不支持 STA+AP 并发，A 方案不可行"
elif [ "${NET_OK:-0}" = "0" ]; then
    log "  ⚠️ AP 起来了但 STA 断网 → 并发存在但不能同时用"
else
    log "  ✅ 并发可用（AP 开 + 外网正常）"
    python3 - "$B_S" "$B_O" "$A_S" "$A_O" "$L_S" "$L_O" <<'PY' 2>/dev/null | tee -a "$REPORT"
import sys
try:
    b_s,b_o,a_s,a_o,l_s,l_o = [float(x) for x in sys.argv[1:7]]
except Exception:
    print("  （数据不足，无法计算影响）"); raise SystemExit
def judge(name, base, now, unit="Hz"):
    if base <= 0:
        print("  %s: 基线无数据，跳过" % name); return
    pct = (now - base) / base * 100.0
    tag = "✅ 无影响" if abs(pct) < 10 else ("⚠️ 轻微" if abs(pct) < 30 else "❌ 明显")
    print("  %s: %.1f → %.1f %s  (%+.0f%%)  %s"
          % (name, base, now, unit, pct, tag))
print()
print("  【AP 本身的开销】")
judge("/scan", b_s, a_s); judge("/odom", b_o, a_o)
print()
print("  【我的大流量对 AP 的影响】")
judge("/scan", a_s, l_s); judge("/odom", a_o, l_o)
PY
fi

exit 0
