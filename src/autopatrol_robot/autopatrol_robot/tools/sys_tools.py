"""
tools/sys_tools.py
AI 可调用的系统状态工具（位置查询、电量、预定义点列表）。
"""

from langchain_core.tools import tool

from autopatrol_robot.tools.nav_tools import LOCATION_MAP
from autopatrol_robot.location_store import get_location_store


@tool
def get_current_position() -> str:
    """获取机器人当前在地图中的位置和朝向。

    当用户问"我在哪"、"当前位置"时使用此工具。
    返回的坐标是 map 坐标系下的 AMCL 定位结果。
    """
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


@tool
def initialize_pose() -> str:
    """重置 AMCL 的位姿估计，强制认为机器人此刻站在地图原点。

    ★ 这是**破坏性**操作：它会立刻把定位跳到 (0,0)，如果车实际不在
    原点，后续导航会全部出错。所以只在明确要求时使用。

    仅当用户**明确**说下面这类话时使用：
      "初始化位姿"、"初始化定位"、"初始化 AMCL"、"重置定位"
      "把当前位置设成原点"

    ⚠️ 不要因为用户说了"起点"二字就调这个！
       "回起点 / 回家 / 返回起点" 是**导航**动作，
       应该用 return_to_start，本工具不负责移动机器人。
    """
    try:
        from autopatrol_robot.navigation_service import get_global_nav_service

        nav_service = get_global_nav_service()
        if nav_service is None:
            return "⚠️ 导航服务未初始化，请先加载地图"

        return nav_service.initialize_amcl_pose()
    except Exception as e:
        return f"❌ 初始化位姿失败: {str(e)}"


@tool
def get_battery_status() -> str:
    """
    获取机器人电池电量。

    当用户问"电量多少"、"还有电吗"时使用。
    """
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
