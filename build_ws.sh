#!/bin/bash
# ============================================================
# chapt9_update ROS2 工作空间编译脚本
# 自动剥离 ARM 交叉编译工具链，避免污染 ROS2 编译
# 用法:
#   ./build_ws.sh                         # 编译全部
#   ./build_ws.sh --packages-select xxx   # 只编译指定包
# ============================================================

# ---- 1. 清理 PATH 中的 ARM 交叉编译工具链 ----
ARM_MARKER="arm-buildroot"
CLEAN_PATH=""
IFS=':' read -ra ADDR <<< "$PATH"
for p in "${ADDR[@]}"; do
    if [[ "$p" != *"$ARM_MARKER"* ]]; then
        [ -z "$CLEAN_PATH" ] && CLEAN_PATH="$p" || CLEAN_PATH="$CLEAN_PATH:$p"
    fi
done
export PATH="$CLEAN_PATH"
echo "✅ ARM 交叉编译工具链已从 PATH 中移除"

# ---- 2. 确保 ROS2 环境（保留 AMENT_PREFIX_PATH / COLCON_PREFIX_PATH） ----
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble 环境已加载"
fi

# ---- 4. 切换到工作空间 ----
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/fishbot_ws"
cd "$WS_DIR" || exit 1
echo "📍 工作空间: $WS_DIR"
echo ""

# ---- 5. 执行编译 ----
# 使用 --continue-on-error 确保一个包失败不影响其他包
colcon build --packages-skip robot_controller_qt --continue-on-error "$@"
