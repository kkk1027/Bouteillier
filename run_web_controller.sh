#!/bin/bash
# ============================================================
# 启动小车网页控制服务 (web_controller.py)
# 用法：
#   ./run_web_controller.sh                    # 自动：有本地模型就用本地
#   FISHBOT_LLM=local    ./run_web_controller.sh    # 强制本地 Ollama
#   FISHBOT_LLM=deepseek ./run_web_controller.sh    # 强制云端 DeepSeek
#   DEEPSEEK_API_KEY=sk-xxx ./run_web_controller.sh # 临时指定 key
# 然后浏览器打开 http://<本机IP>:5000
# ============================================================

set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

# 本地服务绝不能被代理接管（系统里存在 127.0.0.1:17891 代理）
export NO_PROXY="127.0.0.1,localhost,::1${NO_PROXY:+,$NO_PROXY}"
export no_proxy="$NO_PROXY"

# 1. 加载 ROS2 与工作空间环境
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ 未找到 /opt/ros/humble/setup.bash，请先安装 ROS 2 Humble" >&2
    exit 1
fi

if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
else
    echo "❌ 未找到 $WS/install/setup.bash" >&2
    echo "   请先编译工作空间：" >&2
    echo "   cd $WS && colcon build --packages-skip robot_controller_qt micro_ros_agent --symlink-install" >&2
    exit 1
fi

# 2. 检查 AI 模型来源
# ★ 不要在这里注入 sk-placeholder-no-ai。
#   以前脚本会 `export DEEPSEEK_API_KEY="sk-placeholder-no-ai"` 兜底，
#   结果用户明明配了 key，程序却拿着占位值去请求，报
#   「401 Authentication Fails, Your api key: ****o-ai is invalid」——
#   那句里的 `o-ai` 就是 placeholder 的尾巴，极难看出真正原因。
#   现在改为：没有就明确告知，并让 agent_graph.py 从 .env 文件兜底。
#
# 本地模型（Ollama）优先级高于云端，因为它免费且够用（实测 20/20）。
# 想强制走云端：FISHBOT_LLM=deepseek ./run_web_controller.sh
LLM_MODE="${FISHBOT_LLM:-auto}"

if [ "$LLM_MODE" != "deepseek" ]; then
    if [ -x "$WS/scripts/local_llm.sh" ]; then
        if "$WS/scripts/local_llm.sh" start; then
            echo "🖥️  AI 使用本地模型（零 API 费用）"
            # 启动内存看门狗：Ollama 的 prompt cache 会随对话轮数
            # 线性增长（q8_0 下约 36MB/条），本机无 swap，
            # 涨满会导致整机死机（2026-10-31 已发生一次）。
            "$WS/scripts/local_llm.sh" watch >/dev/null 2>&1 || true
        elif [ "$LLM_MODE" = "local" ]; then
            echo "❌ 已指定 FISHBOT_LLM=local 但本地模型启动失败" >&2
            exit 1
        else
            echo "⚠️  本地模型不可用，将回退到 DeepSeek 云 API"
        fi
    fi
fi

if [ -z "$DEEPSEEK_API_KEY" ]; then
    echo "ℹ️  当前 shell 未设置 DEEPSEEK_API_KEY（本地模式下不需要）"
    echo "   如需云端 AI，程序会从以下文件读取（推荐第 1 种）："
    echo "     ~/.config/fishbot/deepseek.env"
    echo "     $WS/.env"
    echo "     $WS/src/autopatrol_robot/.env"
    echo "   设置示例："
    echo "     mkdir -p ~/.config/fishbot"
    echo "     echo 'DEEPSEEK_API_KEY=sk-你的key' > ~/.config/fishbot/deepseek.env"
    echo ""
fi

# 3. 提示访问地址
echo "=========================================="
LOCAL_IPS=$(ip -4 addr show scope global 2>/dev/null \
            | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '^172\.1[7-9]\.' || true)
for ip in $LOCAL_IPS; do
    echo "  访问地址: http://$ip:5000"
done
echo "  本地访问: http://127.0.0.1:5000"
echo "=========================================="

# 4. 启动
exec python3 "$WS/src/autopatrol_robot/autopatrol_robot/web_controller.py"
