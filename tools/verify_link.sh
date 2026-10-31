#!/usr/bin/env bash
# 上下位机连通性自检（在切到 AP 模式后运行）
#
# 为什么需要这个脚本：
#   开 AP 会断掉上位机外网，AI 助手会失去连接，没法边开热点边帮你查。
#   这个脚本把该查的全部查一遍并写进报告文件，切回普通 WiFi 后再回看即可。
#
# 用法：
#   ./tools/verify_link.sh
#
# 报告输出：/tmp/fishbot_link_report.txt
#
# 注意：这里**不能用 set -u**。
# ROS 的 /opt/ros/humble/setup.bash 引用了可能未绑定的变量
# （如 AMENT_TRACE_SETUP_FILES），开了 nounset 后 source 会直接
# 中断整个脚本，表现为「打印到第 3 节就没了」。
set -o pipefail

REPORT=/tmp/fishbot_link_report.txt
DEV=wlp0s20f3
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

: > "$REPORT"
log() { echo "$@" | tee -a "$REPORT"; }
sec() { echo | tee -a "$REPORT"; echo "===== $* =====" | tee -a "$REPORT"; }

log "fishbot 连通性自检   $(date '+%Y-%m-%d %H:%M:%S')"
log "工作空间: $WS"

# ---------------------------------------------------------------- 1
sec "1. 网卡与 AP 状态"
nmcli -t -f DEVICE,STATE,CONNECTION device 2>/dev/null | grep "$DEV" | tee -a "$REPORT"
ip -4 addr show "$DEV" 2>/dev/null | grep -oP 'inet \K[0-9.]+/\d+' | tee -a "$REPORT"
if nmcli -t -f NAME connection show --active 2>/dev/null | grep -qx fishbot-ap; then
    log "  ✅ fishbot-ap 已激活"
else
    log "  ❌ fishbot-ap 未激活 —— 先执行 sudo nmcli con up fishbot-ap"
fi

# ---------------------------------------------------------------- 2
sec "2. AP 已连设备（看两块板有没有连上）"
ip neigh show dev "$DEV" 2>/dev/null | grep -v FAILED | tee -a "$REPORT" \
    || log "  （无邻居记录）"
log ""
log "  说明：应能看到 2 个 172.20.10.x 的设备"
log "  ESP32 主板的 MAC 前缀是 14:08:08（本例 14:08:08:a2:46:f8）"
log "  雷达转接板 MAC 是 60:01:94:7a:c8:6a"

# ---------------------------------------------------------------- 3
sec "3. ROS2 话题（下位机是否在发布）"
if command -v ros2 >/dev/null 2>&1 || [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash 2>/dev/null
    # shellcheck disable=SC1090
    source "$WS/install/setup.bash" 2>/dev/null

    for t in /odom /imu_raw /scan /battery_voltage; do
        hz=$(timeout 5 ros2 topic hz "$t" 2>/dev/null | grep -m1 average \
             | grep -oP 'average rate: \K[0-9.]+')
        if [ -n "$hz" ]; then
            log "  ✅ $t   ${hz} Hz"
        else
            log "  ❌ $t   无数据"
        fi
    done
else
    log "  ⚠️ 找不到 ROS2 环境"
fi

# ---------------------------------------------------------------- 4
sec "4. 端口监听"
log "  micro_ros_agent UDP 8888: $(ss -lunp 2>/dev/null | grep -c ':8888')"
log "  TCP 桥接 8889:            $(ss -ltnp 2>/dev/null | grep -c ':8889')"
log "  web 5000:                 $(ss -ltn  2>/dev/null | grep -c ':5000')"

# ---------------------------------------------------------------- 5
sec "5. 关键日志（最近 30 行）"
for f in "$WS/log/web/bringup.log" "$WS/log/web/navigation2.log"; do
    if [ -f "$f" ]; then
        log "--- $f ---"
        grep -aE "micro-ROS|microROS|来自.*的连接|TCP connected|桥接统计|WiFi" \
            "$f" 2>/dev/null | tail -12 | tee -a "$REPORT"
    fi
done

# ---------------------------------------------------------------- 6
sec "6. 重连测试指引"
log "  最关键的验证：模拟「上位机断连」"
log "    1) 新开一个终端，杀掉 agent："
log "         pkill -f micro_ros_agent"
log "    2) 观察 ESP32 串口（115200）："
log "         应出现  [microROS] Agent 失联，销毁实体准备重连"
log "         然后    [microROS] 重连成功（第 N 次恢复）"
log "         且**不应出现 rst:0x1 重启**，odom 不应归零"
log "    3) 重新拉起 agent："
log "         ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
log ""
log "  雷达板重连测试："
log "    杀掉 tcp 桥接再重启，雷达板日志应出现"
log "      TCP connected to 172.20.10.5:8889"

sec "完成"
log "报告已写入 $REPORT"
echo
echo "======================================================"
cat "$REPORT"
