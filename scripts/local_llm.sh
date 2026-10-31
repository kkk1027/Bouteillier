#!/bin/bash
# ============================================================
# 本地大模型（Ollama）管理脚本
# ------------------------------------------------------------
#   ./scripts/local_llm.sh start    启动服务（已在跑则跳过）
#   ./scripts/local_llm.sh stop     停止服务
#   ./scripts/local_llm.sh status   查看状态（含显存占用）
#   ./scripts/local_llm.sh test     跑一遍工具调用自检
#   ./scripts/local_llm.sh warm     预热模型（提前占住显存）
# ============================================================
set -u

OLLAMA_BIN="$HOME/.local/ollama/bin/ollama"
MODEL="${FISHBOT_LOCAL_MODEL:-fishbot-qwen:3b}"
OLLAMA_HOST_URL="${OLLAMA_HOST:-127.0.0.1:11434}"
LOG="/tmp/ollama.log"

# 本地服务绝不能被代理接管
export NO_PROXY="127.0.0.1,localhost,::1${NO_PROXY:+,$NO_PROXY}"
export no_proxy="$NO_PROXY"

start() {
    if curl -s -m 2 -o /dev/null "http://$OLLAMA_HOST_URL/api/tags"; then
        echo "✅ Ollama 已在运行（$OLLAMA_HOST_URL）"
        return 0
    fi
    if [ ! -x "$OLLAMA_BIN" ]; then
        echo "❌ 未找到 $OLLAMA_BIN"
        echo "   安装方法见 scripts/LOCAL_LLM.md"
        return 1
    fi

    echo "🚀 启动 Ollama ..."
    export OLLAMA_HOST="$OLLAMA_HOST_URL"
    export OLLAMA_MODELS="${OLLAMA_MODELS:-$HOME/.local/ollama/models}"
    export OLLAMA_KEEP_ALIVE="${OLLAMA_KEEP_ALIVE:-10m}"

    # ★ 显存保护（2026-10-31 死机事故的教训）
    #   RTX 3050 Laptop 只有 4GB 显存，Xorg/GNOME 常驻约 600MB。
    #   之前用 num_ctx=8192 把显存顶到上限，内核报 8 次
    #   NV_ERR_NO_MEMORY，GPU 驱动重新初始化后整机硬死机。
    #   GPU_OVERHEAD 预留 512MB 给桌面环境，避免驱动分配失败。
    export OLLAMA_GPU_OVERHEAD="${OLLAMA_GPU_OVERHEAD:-536870912}"
    #   只允许同时加载 1 个模型，防止切换模型时两份权重同时占显存
    export OLLAMA_MAX_LOADED_MODELS="${OLLAMA_MAX_LOADED_MODELS:-1}"

    # ★ KV cache 量化（2026-10-31 死机事故的第二个教训）
    #   Ollama 会为「每一个不同的 prompt」缓存一份完整 KV cache，
    #   上限按内存推算，本机高达 8192 MiB。f16 时每个条目约 70MB，
    #   聊 100 多轮就能吃满 7GB → 系统内存耗尽 → 死机。
    #   q8_0 把每条目减半到约 36MB，实测准确率不变（95.8~100%）。
    export OLLAMA_KV_CACHE_TYPE="${OLLAMA_KV_CACHE_TYPE:-q8_0}"

    # 注意：这里故意不设置 HTTPS_PROXY。
    #   * 拉模型(pull)才需要代理，start 不需要；
    #   * 若把代理写进 serve 的环境，本地 127.0.0.1 请求会绕一圈代理，
    #     在部分代理实现下会被拒绝。
    nohup "$OLLAMA_BIN" serve > "$LOG" 2>&1 &

    for i in $(seq 1 40); do
        if curl -s -m 2 -o /dev/null "http://$OLLAMA_HOST_URL/api/tags"; then
            echo "✅ Ollama 已就绪（${i}00ms）"
            return 0
        fi
        sleep 0.1
    done
    echo "❌ Ollama 启动超时，日志：$LOG"
    tail -20 "$LOG"
    return 1
}

stop() {
    pkill -f "ollama serve" 2>/dev/null && echo "🛑 已停止 Ollama" || echo "Ollama 未在运行"
}

status() {
    if curl -s -m 2 -o /dev/null "http://$OLLAMA_HOST_URL/api/tags"; then
        echo "服务状态: ✅ 运行中  ($OLLAMA_HOST_URL)"
    else
        echo "服务状态: ❌ 未运行"
        return 1
    fi
    echo ""
    echo "已安装模型:"
    "$OLLAMA_BIN" list 2>/dev/null | sed 's/^/  /'
    echo ""
    echo "运行中的模型:"
    "$OLLAMA_BIN" ps 2>/dev/null | sed 's/^/  /'
    if command -v nvidia-smi >/dev/null 2>&1; then
        echo ""
        echo "显存占用:"
        nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader \
            | sed 's/^/  /'
    fi
}

warm() {
    curl -s -m 300 "http://$OLLAMA_HOST_URL/api/generate" \
        -d "{\"model\":\"$MODEL\",\"prompt\":\"你好\",\"stream\":false}" \
        -o /dev/null && echo "✅ 模型 $MODEL 已预热并常驻显存"
}

# ============================================================
# 内存保护：Ollama 的 prompt cache 会随对话轮数线性增长
# ------------------------------------------------------------
# 已证实：每个「不同的 prompt」都缓存一份完整 KV cache。
#   f16  num_ctx=4096 → 约 70MB/条
#   q8_0 num_ctx=4096 → 约 36MB/条
# 上限按内存推算（本机约 8GB），涨满就把系统内存吃干净。
# 本机 15GB 内存、**无 swap**，一旦耗尽内核连 order-9 页都分配
# 不出来，GPU 驱动随之报 NV_ERR_NO_MEMORY，最终整机硬死机。
#
# 这里的做法：超过阈值就把模型卸载掉。卸载会释放全部缓存和显存，
# 代价只是下一次请求要重新加载模型（约 10~40 秒）。
# ============================================================
guard() {
    local limit_mb="${FISHBOT_LLM_RSS_LIMIT_MB:-3500}"
    local pid rss
    pid="$(pgrep -f "ollama/lib/ollama/llama-server" | head -1)"

    if [ -z "$pid" ]; then
        echo "ℹ️  模型未加载（无缓存占用）"
        return 0
    fi

    rss=$(awk '/^Rss:/{printf "%d", $2/1024}' "/proc/$pid/smaps_rollup" 2>/dev/null)
    rss="${rss:-0}"
    local avail
    avail=$(free -m | awk '/^内存|^Mem/{print $7}')

    if [ "$rss" -gt "$limit_mb" ]; then
        echo "⚠️  模型内存 ${rss}MB 超过阈值 ${limit_mb}MB，卸载以释放缓存"
        curl -s -m 10 "http://$OLLAMA_HOST_URL/api/generate" \
            -d "{\"model\":\"$MODEL\",\"keep_alive\":0}" -o /dev/null
        sleep 3
        echo "✅ 已卸载。系统可用内存：$(free -m | awk '/^内存|^Mem/{print $7}')MB"
        return 0
    fi

    if [ "$avail" -lt 1500 ]; then
        echo "⚠️  系统可用内存仅 ${avail}MB，主动卸载模型"
        curl -s -m 10 "http://$OLLAMA_HOST_URL/api/generate" \
            -d "{\"model\":\"$MODEL\",\"keep_alive\":0}" -o /dev/null
        sleep 3
        echo "✅ 已卸载。系统可用内存：$(free -m | awk '/^内存|^Mem/{print $7}')MB"
        return 0
    fi

    echo "✅ 模型内存 ${rss}MB / 阈值 ${limit_mb}MB，系统可用 ${avail}MB，正常"
}

unload() {
    curl -s -m 10 "http://$OLLAMA_HOST_URL/api/generate" \
        -d "{\"model\":\"$MODEL\",\"keep_alive\":0}" -o /dev/null \
        && echo "✅ 已卸载 $MODEL，缓存与显存已释放"
}

# ============================================================
# 后台看门狗：每 60 秒执行一次 guard
# ------------------------------------------------------------
# 目的是把「聊天久了内存涨满 → 死机」拦在发生之前。
# guard 只在真的超阈值时才卸载模型，平时只是一次内存读取，
# 开销可以忽略。
# ============================================================
WATCHDOG_PIDFILE="/tmp/fishbot_llm_watchdog.pid"

watch_start() {
    if [ -f "$WATCHDOG_PIDFILE" ] && kill -0 "$(cat "$WATCHDOG_PIDFILE")" 2>/dev/null; then
        echo "✅ 看门狗已在运行（PID $(cat "$WATCHDOG_PIDFILE")）"
        return 0
    fi
    (
        SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
        while true; do
            sleep 60
            "$SELF" guard >> /tmp/fishbot_llm_guard.log 2>&1
        done
    ) &
    echo $! > "$WATCHDOG_PIDFILE"
    echo "✅ 看门狗已启动（PID $!），每 60 秒检查一次，日志 /tmp/fishbot_llm_guard.log"
}

watch_stop() {
    if [ -f "$WATCHDOG_PIDFILE" ]; then
        kill "$(cat "$WATCHDOG_PIDFILE")" 2>/dev/null
        rm -f "$WATCHDOG_PIDFILE"
        echo "🛑 看门狗已停止"
    else
        echo "看门狗未在运行"
    fi
}

case "${1:-start}" in
    start)  start ;;
    stop)   stop ;;
    status) status ;;
    warm)   warm ;;
    guard)  guard ;;
    unload) unload ;;
    watch)  watch_start ;;
    watch-stop) watch_stop ;;
    restart) stop; sleep 1; start ;;
    test)
        cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
        python3 src/autopatrol_robot/autopatrol_robot/tools/local_llm_selftest.py
        ;;
    *)
        echo "用法: $0 {start|stop|status|warm|guard|unload|watch|watch-stop|restart|test}"
        exit 1
        ;;
esac
