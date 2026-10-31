#!/usr/bin/env bash
# =====================================================================
# fishbot 链路只读诊断
# ---------------------------------------------------------------------
# 安全声明：本脚本【只读】。不启动节点、不发布任何指令，
#           不会产生 /cmd_vel，不会让小车动。
#
# 用途：定位「/scan 消失」「AMCL 不收敛」「导航不动」这类问题的层次。
#       把雷达链路的每一段都测一遍，直接指出断在哪一段。
# 用法：
#   ./diag_chain.sh            # 默认 ROS_DOMAIN_ID=0（实车）
#   ROS_DOMAIN_ID=77 ./diag_chain.sh
# =====================================================================
set -o pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR" || exit 1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ROS 的 setup.bash 引用了未定义变量，不能开 set -u（会直接退出）
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null
# shellcheck disable=SC1091
source install/setup.bash 2>/dev/null

OK="✅"; BAD="❌"; WARN="⚠️ "
FAILED=0

RATE_TOOL="$WS_DIR/topic_rate.py"

# ★ 注意 pipefail：本脚本开了 set -o pipefail，而几乎每个探测都用
#   `timeout N cmd | grep -q xxx` 的形式。timeout 到时杀掉 cmd 会返回
#   124，pipefail 会把整条管道的状态也变成 124 —— 于是 grep 明明匹配
#   成功也被判成失败。项目实际踩过：TF 明明正常却报"查不到"。
#   所以凡是靠 timeout 结束的探测，必须先把输出落到变量/文件再判断。
probe() {  # probe <秒> <命令...>  → 输出回显，状态恒为 0
  local secs="$1"; shift
  timeout "$secs" "$@" 2>&1 || true
}

# 用 QoS 自适应的探活工具测话题。
# 千万不要用 ros2 topic hz：它默认 RELIABLE 订阅，而 ESP32(micro-ROS)
# 发的 /odom、/imu_raw 是 BEST_EFFORT，两者 QoS 不兼容 → 有数据也报
# "no new messages"，把人往"ESP32 没连上"的错误方向引（实际踩过）。
# 返回：OK ... / EMPTY / NOTOPIC / ERROR ...
rate() {
  timeout 60 python3 "$RATE_TOOL" "$1" "${2:-3}" 2>/dev/null | grep -v RTPS | tail -1
}

# 把 rate() 的结果转成一行结论
check_topic() {
  local topic="$1" need="$2" res
  res="$(rate "$topic" "${3:-3}")"
  case "$res" in
    OK*)
      local hz
      hz="$(echo "$res" | sed -n 's/.*hz=\([0-9.]*\).*/\1/p')"
      if [ -n "$need" ] && awk "BEGIN{exit !($hz < $need)}" 2>/dev/null; then
        warn "$topic 有数据但偏慢：$(echo "$res" | sed 's/^OK //')（期望 >=${need}Hz）"
      else
        pass "$topic $(echo "$res" | sed 's/^OK //')"
      fi
      ;;
    EMPTY*)   fail "$topic 3 秒内无任何数据（话题存在但没人发）" ;;
    NOTOPIC*) fail "$topic 话题不存在" ;;
    *)        warn "$topic 探测异常: $res" ;;
  esac
}

say()  { printf '%s\n' "$*"; }
head1() { printf '\n\033[1m=== %s ===\033[0m\n' "$*"; }
pass() { printf '%s %s\n' "$OK" "$*"; }
fail() { printf '%s %s\n' "$BAD" "$*"; FAILED=$((FAILED+1)); }
warn() { printf '%s %s\n' "$WARN" "$*"; }

# ---------------------------------------------------------------------
head1 "0. 环境"
say "ROS_DOMAIN_ID = $ROS_DOMAIN_ID   （实车必须为 0，离线测试常被改成 77~120）"
say "工作空间     = $WS_DIR"
if [ "$ROS_DOMAIN_ID" != "0" ]; then
  warn "当前不是 0 号域，下面很可能全部查不到实车节点"
fi

# ---------------------------------------------------------------------
head1 "0.5 网络：是否连在小车那个热点上"
# 实测踩过：电脑切到了家里 WiFi(192.168.31.x)，而 ESP32(172.20.10.2) 与
# 雷达转接板(172.20.10.3) 还在手机热点上。表现为 TCP 连接看似还挂着
# （其实是旧连接残留）但一个字节都过不来，ROS 节点也互相看不见。
MY_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
say "本机 IP = ${MY_IP:-<无>}"
if echo "$MY_IP" | grep -q '^172\.20\.10\.'; then
  pass "已在手机热点网段 172.20.10.x"
else
  warn "本机不在 172.20.10.x —— 很可能没连小车热点"
  say "     小车侧固定地址：ESP32 172.20.10.2 / 雷达转接板 172.20.10.3"
  say "     换回热点后本机 IP 必须也是 172.20.10.x，否则改固件或改热点"
fi
for ip in 172.20.10.2 172.20.10.3; do
  if timeout 3 ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then
    pass "ping $ip 通"
  else
    fail "ping $ip 不通（对应设备不在线，或热点不对）"
  fi
done

# ---------------------------------------------------------------------
head1 "1. tcp_server 与雷达转接板"
# 必须按「谁在监听 8889」选进程，不能简单取第一个 —— 开发调试时
# 可能还开着别的 tcp_server（不同端口），选错会得到完全错误的结论。
# 本项目实际踩过：选了被落在 8901 的测试进程，把正常链路报成故障。
TPID="$(ss -tlnp 2>/dev/null | grep ':8889 ' \
        | grep -o 'pid=[0-9]*' | head -1 | cut -d= -f2)"
if [ -z "$TPID" ]; then
  TPID="$(pgrep -f 'ros_serial2wifi/lib/ros_serial2wifi/tcp_server' | head -1)"
fi
if [ -z "$TPID" ]; then
  fail "tcp_server 没在运行（8889 无人监听，转接板永远连不上）"
else
  # 提醒同时存在多个 tcp_server 的情况
  ALL_TCP="$(pgrep -f 'ros_serial2wifi/lib/ros_serial2wifi/tcp_server' | wc -l)"
  if [ "$ALL_TCP" -gt 1 ]; then
    warn "检测到 $ALL_TCP 个 tcp_server 进程，只分析占用 8889 的那个"
  fi
  pass "tcp_server 在运行 (pid=$TPID)"

  LISTEN="$(ss -tlnp 2>/dev/null | grep -c ':8889 ')"
  [ "$LISTEN" -gt 0 ] && pass "8889 正在监听" || fail "8889 未监听"

  PEER="$(ss -tnp 2>/dev/null | grep ':8889 ' | grep ESTAB | head -1)"
  if [ -n "$PEER" ]; then
    PEER_IP="$(echo "$PEER" | awk '{print $5}' | cut -d: -f1)"
    if timeout 3 ping -c 1 -W 1 "$PEER_IP" >/dev/null 2>&1; then
      pass "转接板已连入：$PEER_IP"
      # 采样 3 秒，看「雷达→本端」方向的字节增量
      R1="$(awk '/^rchar/{print $2}' "/proc/$TPID/io" 2>/dev/null)"
      sleep 3
      R2="$(awk '/^rchar/{print $2}' "/proc/$TPID/io" 2>/dev/null)"
      DELTA=$(( ${R2:-0} - ${R1:-0} ))
      if [ "${DELTA:-0}" -gt 3000 ]; then
        pass "3 秒收到 ${DELTA} 字节（≈$((DELTA/3)) B/s，雷达在出数）"
      elif [ "$DELTA" -gt 0 ]; then
        warn "3 秒仅 ${DELTA} 字节，速率异常偏低（雷达 5Hz 正常应 >10KB/s）"
      else
        fail "3 秒内 0 字节 —— 转接板 TCP 连着但不转发，属转接板假死"
        say "     处理：给雷达转接板断电重上电；tcp_server 也会在"
        say "           idle_disconnect_sec(默认5s) 后自动断开逼它重连"
      fi
      SQ="$(echo "$PEER" | awk '{print $3}')"
      if [ "${SQ:-0}" -gt 0 ]; then
        fail "socket 发送队列积压 $SQ 字节（对端不收，链路可能已卡死）"
      fi
    else
      # 内核还挂着 ESTABLISHED，但对端根本不可达 —— 断网后的残留连接。
      # 别当成「在线」，否则会白查很久硬件。
      fail "转接板 $PEER_IP 显示 ESTABLISHED，但 ping 不通 —— 残留假连接"
      say "     原因不在桥接代码，是网络已断：重新连到小车热点即可"
      say "     （旧进程不会自己恢复，重连热点后请重启 launch）"
    fi
  else
    fail "转接板未连入 8889（转接板没上电 / 不在同一热点 / 固件里的 IP 不是本机）"
    say "     本机 IP：$(hostname -I 2>/dev/null | awk '{print $1}')"
    say "     转接板应该连这个 IP 的 8889 端口"
  fi
fi

# ---------------------------------------------------------------------
head1 "2. 虚拟串口 /tmp/tty_laser"
if [ -L /tmp/tty_laser ]; then
  pass "/tmp/tty_laser -> $(readlink /tmp/tty_laser)"
else
  fail "/tmp/tty_laser 不存在（tcp_server 没起来，或起在了别的 serial_port）"
fi

# ---------------------------------------------------------------------
head1 "3. ydlidar 驱动"
YPID="$(pgrep -f 'ydlidar/lib/ydlidar/ydlidar_node' | head -1)"
if [ -n "$YPID" ]; then
  pass "ydlidar_node 在运行 (pid=$YPID)"
else
  fail "ydlidar_node 没在运行"
  # 去最新的 launch.log 里找它为什么起不来。这一步非常关键：
  # 2026-10-30 踩过 —— bringup 里 voice_control.launch.py 抢占了同名
  # 的 params_file，ydlidar 拿着 voice_control.yaml 启动，port 回落到
  # /dev/ydlidar，0.5ms 秒退，respawn 重试 1000+ 次。
  LATEST_LOG="$(ls -td ~/.ros/log/*/ 2>/dev/null | head -1)"
  if [ -n "$LATEST_LOG" ] && [ -f "${LATEST_LOG}launch.log" ]; then
    YD_ERR="$(grep -a "ydlidar_node" "${LATEST_LOG}launch.log" 2>/dev/null | tail -3)"
    if [ -n "$YD_ERR" ]; then
      say "     最近日志（${LATEST_LOG}launch.log）："
      echo "$YD_ERR" | sed 's/^/       /' | cut -c1-150
      # 参数文件是否用错 —— 这是最阴的一个坑
      BAD_PARAM="$(echo "$YD_ERR" | grep -o -- '--params-file [^ ]*' \
                  | sed "s/'\].*//" | grep -v 'ydlidar.yaml' | head -1)"
      if [ -n "$BAD_PARAM" ]; then
        fail "参数文件用错了：$BAD_PARAM"
        say "     原因：launch 参数名冲突（别的包也声明了 params_file）。"
        say "     确认 ydlidar_launch.py 用的是 ydlidar_params_file。"
        say "     该文件里没有 ydlidar_node 段时，port 会回落 /dev/ydlidar"
        say "     而物理设备是 /tmp/tty_laser —— 必然打不开。"
      fi
    fi
  fi
  say "     单独看报错：ros2 launch ydlidar ydlidar_launch.py"
fi

# 正在跑的桥接进程是否比源码旧（Python 不会热更新，改了必须重启）
TPID2="$(pgrep -f 'ros_serial2wifi/lib/ros_serial2wifi/tcp_server' | head -1)"
SRC_TCP="$WS_DIR/src/ros_serial2wifi/ros_serial2wifi/tcpserver.py"
if [ -n "$TPID2" ] && [ -f "$SRC_TCP" ] && [ -r "/proc/$TPID2/stat" ]; then
  # 用 /proc/<pid> 目录的创建时间近似进程启动时间
  PROC_START=$(stat -c %Y "/proc/$TPID2" 2>/dev/null || echo 0)
  SRC_MTIME=$(stat -c %Y "$SRC_TCP" 2>/dev/null || echo 0)
  if [ "${PROC_START:-0}" -lt "${SRC_MTIME:-0}" ]; then
    warn "tcp_server 进程比 tcpserver.py 旧 —— 跑的是旧代码，改动未生效"
    say "     Python 不会热更新已加载的模块，改完必须重启 bringup"
  fi
fi

# ---------------------------------------------------------------------
head1 "4. /scan 话题"
SCAN_RES="$(rate /scan 3)"
case "$SCAN_RES" in
  OK*)    pass "/scan 有数据：$(echo "$SCAN_RES" | sed 's/^OK //')" ;;
  EMPTY*) fail "/scan 3 秒内无数据（驱动在跑但没吐帧，看第 3 项的日志）" ;;
  *)      fail "/scan 探测失败：$SCAN_RES" ;;
esac
NPUB="$(timeout 8 ros2 topic info /scan 2>/dev/null | grep -i 'Publisher count' | awk '{print $3}')"
[ -n "$NPUB" ] && say "     Publisher count = $NPUB"

# ---------------------------------------------------------------------
head1 "5. 底盘与融合里程计"
# ESP32 的 /odom、/imu_raw 是 BEST_EFFORT，用 rate() 自动匹配 QoS
check_topic /odom 20 3
check_topic /imu_raw 20 3

# EKF 只在 navigation2 / manual_mapping 里启动，bringup 本身不启动。
# 所以单跑 bringup 时 /odometry/filtered 为空是**预期行为**，不是故障。
FILT_RES="$(rate /odometry/filtered 3)"
if echo "$FILT_RES" | grep -q '^OK'; then
  pass "/odometry/filtered $(echo "$FILT_RES" | sed 's/^OK //')"
else
  if pgrep -f 'ekf_node' >/dev/null 2>&1; then
    fail "EKF 进程在，但 /odometry/filtered 无输出（看 EKF 日志）"
  else
    warn "/odometry/filtered 无数据 —— EKF 未启动（属正常）"
    say "     EKF 只在 navigation2.launch.py / manual_mapping.launch.py"
    say "     里启动，bringup 不含 EKF。要拿 odom→base_footprint 需起导航或建图。"
  fi
fi

# ---------------------------------------------------------------------
head1 "6. TF 链"
# 静态 TF（base_footprint→base_link→laser_link）由 urdf2tf 的
# robot_state_publisher 发布，bringup 就会起，必须正常。
tf_ok() {  # tf_ok <target> <source>
  probe 10 ros2 run tf2_ros tf2_echo "$1" "$2" | grep -q 'At time'
}
for pair in "base_footprint base_link" "base_link laser_link"; do
  set -- $pair
  if tf_ok "$1" "$2"; then
    pass "TF $1 -> $2 正常"
  else
    fail "TF $1 -> $2 查不到（urdf2tf / robot_state_publisher 异常）"
  fi
done

# 重复发布同一静态 TF 会让下游报 TF_REPEATED_DATA 刷屏，
# 典型成因是 bringup 与导航 launch 各起了一份 robot_state_publisher。
RSP_CNT="$(pgrep -fc 'robot_state_publisher' 2>/dev/null || echo 0)"
if [ "${RSP_CNT:-0}" -gt 2 ]; then
  warn "有 $RSP_CNT 个 robot_state_publisher（正常应为 2：1个主进程+1个子线程？）"
  say "     若 >2，说明有残留进程重复发布静态 TF，建议全部重启"
fi

# odom->base_footprint 由 odom2tf 发，而 odom2tf 订阅 /odometry/filtered，
# 后者由 EKF 产出。EKF 不在 bringup 里。所以要分情况说清楚，
# 否则会让人误以为是定位坏了。
if tf_ok odom base_footprint; then
  pass "TF odom -> base_footprint 正常"
elif pgrep -f 'ekf_node' >/dev/null 2>&1; then
  fail "TF odom -> base_footprint 缺失，但 EKF 在跑（odom2tf 或 EKF 配置问题）"
else
  warn "TF odom -> base_footprint 缺失 —— EKF 未启动，这是预期的"
  say "     它依赖链：/odom → 协方差注入 → EKF → /odometry/filtered → odom2tf"
  say "     启导航或建图 launch 后才会出现。"
fi

# ---------------------------------------------------------------------
head1 "7. 地图与 AMCL"
# /map 是 RELIABLE + TRANSIENT_LOCAL，rate() 会自动匹配到正确 QoS
MAP_RES="$(rate /map 3)"
if echo "$MAP_RES" | grep -q '^OK'; then
  pass "/map 可获取"
else
  if pgrep -f 'map_server' >/dev/null 2>&1; then
    fail "/map 无数据，但 map_server 在跑（需 configure+activate）"
  else
    warn "/map 无数据 —— map_server 未启动（需跑导航 launch）"
  fi
fi

if probe 12 ros2 node list | grep -qx '/amcl'; then
  pass "AMCL 节点存在"
  AMCL_RES="$(rate /amcl_pose 3)"
  if echo "$AMCL_RES" | grep -q '^OK'; then
    pass "/amcl_pose $(echo "$AMCL_RES" | sed 's/^OK //')"
  else
    warn "/amcl_pose 暂无输出 —— AMCL 还没定位成功（需激光 + 地图 + 一点运动）"
  fi
else
  warn "AMCL 未启动（需跑 navigation2 launch）"
fi

# ---------------------------------------------------------------------
head1 "结论"
if [ "$FAILED" -eq 0 ]; then
  say "$OK 链路全部正常。若 AMCL 仍不收敛，请看 auto_localizer 日志的协方差，"
  say "   并缓慢原地转动小车（静止时 AMCL 几乎不会收敛）。"
else
  say "$BAD 有 $FAILED 项异常，按上面从上到下的顺序修 —— 上层依赖下层，"
  say "   前面的不修好，后面的必然连带失败。"
fi
exit 0
