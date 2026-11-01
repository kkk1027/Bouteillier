"""
tools/sys_tools.py
AI 可调用的系统状态工具（位置查询、电量、预定义点列表）。
"""

from langchain_core.tools import tool

from autopatrol_robot.tools.nav_tools import LOCATION_MAP
from autopatrol_robot.location_store import get_location_store


@tool
def get_current_position() -> str:
    """获取当前位置和朝向。用户说我在哪、现在在哪时用。"""
    try:
        from autopatrol_robot.navigation_service import get_global_nav_service

        nav_service = get_global_nav_service()
        if nav_service is None:
            return "⚠️ 导航服务未初始化，请先在网页点「载入地图」"

        # detailed=True 能拿到失败的具体原因，而不是含糊的
        # 「AMCL 可能未收敛」——那条提示害得 AI 一直让用户去初始化位姿。
        pos, reason = nav_service.get_current_position(detailed=True)
        if pos is None:
            return (
                "⚠️ 未能获取当前位置。具体原因：%s\n"
                "若提示未收到 /amcl_pose，通常是雷达无数据导致 AMCL 无法"
                "定位；请确认 /scan 有帧（./diag_chain.sh 第 4 项）"
                % (reason or '未知')
            )

        yaw_deg = pos['yaw'] * 57.29578
        return (
            f"📍 当前位置: "
            f"x={pos['x']:.2f}米, y={pos['y']:.2f}米, "
            f"朝向={pos['yaw']:.2f}弧度（约 {yaw_deg:.1f}°）"
            f"　[来源: {reason}]"
        )
    except Exception as e:
        return f"❌ 获取位置失败: {str(e)}"


# ============================================================
# ★ 2026-11-01：`initialize_pose` 工具已**彻底删除**
# ------------------------------------------------------------
# 用户原话：「要不你直接把重定位回原点的工具删了吧，回起点就只会调
#          有关起点的『回到起点』这个唯一工具」
#
# 删除理由（三次实机事故的正本清源）：
#   这个工具会把 AMCL 位姿**直接重置到 (0,0,3.14)**。只要它还存在于
#   工具列表里，3B 模型就有机会在用户说「回起点」时选中它 —— 因为
#   「起点」二字和「回到起点」高度相似。
#
#   前两次修法都是"打补丁"：
#     1. 改工具描述、加 _EXCLUSIVE_NAV_TOOLS 优先级   → 仍有漏网
#     2. 加 _guard_initialize_pose 语义防护           → 仍有漏网
#   根本问题是：只要这个危险工具有机会被选中，就得永远维护一层防护。
#   直接删掉，问题从源头消失。
#
# 人工需要重置 AMCL 时怎么办：
#   navigation_service.initialize_amcl_pose() 方法**仍然保留**，
#   随时可以接成 HTTP 接口或在 RViz 里点 2D Pose Estimate。
#   只是它不再暴露给 AI 了。
# ============================================================

@tool
def get_battery_status() -> str:
    """查询电池电量。用户问电量多少/还有电吗时用。"""
    return "🔋 电池状态: 正常（详细电量请检查硬件话题）"


@tool
def list_predefined_points() -> str:
    """
    列出所有预定义目标点的坐标。

    当用户问"有哪些预定义点"、"预置目标有哪些"时使用。
    """
    store = get_location_store()
    result = "📌 预定义目标点列表:\n"
    for name, idx in LOCATION_MAP.items():
        coords = store.get_predefined_coords(idx)
        if coords:
            result += f"  {idx}. {name}: ({coords[0]:.2f}, {coords[1]:.2f}), 朝向={coords[2]:.2f}\n"
    return result
