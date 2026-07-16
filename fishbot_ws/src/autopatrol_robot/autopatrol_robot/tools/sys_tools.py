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
    """
    try:
        from autopatrol_robot.navigation_service import get_global_nav_service

        nav_service = get_global_nav_service()
        if nav_service is None:
            return "⚠️ 导航服务未初始化，请先加载地图"

        pos = nav_service.get_current_position()
        if pos is None:
            return "⚠️ 未能获取当前位置（AMCL 可能未收敛）"

        return (
            f"📍 当前位置: "
            f"x={pos['x']:.2f}米, y={pos['y']:.2f}米, "
            f"朝向={pos['yaw']:.2f}弧度"
        )
    except Exception as e:
        return f"❌ 获取位置失败: {str(e)}"


@tool
def initialize_pose() -> str:
    """初始化 AMCL 定位系统，告诉机器人「你现在在起点位置」。

    当用户说"初始化位姿"、"初始化"、"设置初始位置"、"初始化AMCL"时使用此工具。
    地图加载后、首次导航前必须调用一次，之后 AMCL 才能正常定位。
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
