"""
tools/loc_tools.py
AI 可调用的地点管理工具（全部通过 LocationStore 操作 patrol_config.yaml）。
"""

from langchain_core.tools import tool
from pydantic import BaseModel, Field

from autopatrol_robot.location_store import get_location_store, NUM_PREDEFINED
from autopatrol_robot.tools.nav_tools import LOCATION_MAP
from autopatrol_robot.tools.sys_tools import get_current_position

# ============================================================
# 参数模型
# ============================================================
class SaveLocationInput(BaseModel):
    name: str = Field(..., description="地点名称，例如：厕所、书房、阳台等")


class NavigateToSavedInput(BaseModel):
    name: str = Field(..., description="已保存的地点名称，如：厕所、书房")


class DeleteSavedLocationInput(BaseModel):
    name: str = Field(..., description="要删除的地点名称，如：厕所、书房")


# ============================================================
# 工具定义
# ============================================================

@tool
def list_locations() -> str:
    """
    列出所有可用地点（预定义 + 自定义）。

    当用户问"有哪些地点"、"可以去哪里"时使用此工具。
    """
    store = get_location_store()
    custom = store.list_all()

    result = "📌 可用地点列表:\n"

    # 预定义地点
    result += "\n【预定义】\n"
    for name, idx in LOCATION_MAP.items():
        coords = store.get_predefined_coords(idx)
        if coords:
            result += f"  • {name}: ({coords[0]:.2f}, {coords[1]:.2f})\n"

    # 自定义地点
    if custom:
        result += "\n【自定义】\n"
        for name, coords in custom.items():
            result += f"  • {name}: ({coords[0]:.2f}, {coords[1]:.2f})\n"
    else:
        result += "\n【自定义】暂无\n"

    return result


@tool(args_schema=SaveLocationInput)
def save_current_location(name: str) -> str:
    """
    将机器人当前位置保存为指定名称的自定义地点。

    当用户说"这里是厕所"、"把这里记作书房"时使用此工具。
    """
    # ★ 直接向 NavigationService 要结构化坐标，不再解析中文文本。
    #   原实现是 `get_current_position.invoke({})` 然后用正则从
    #   "📍 当前位置: x=1.23米..." 里抠数字，一旦提示文案改动就全部失效，
    #   属于典型的「靠字符串契约」耦合。
    try:
        from autopatrol_robot.navigation_service import get_global_nav_service

        nav_service = get_global_nav_service()
        if nav_service is None:
            return "❌ 无法保存地点：导航服务未初始化，请先在网页点「载入地图」"

        pos, reason = nav_service.get_current_position(detailed=True)
        if pos is None:
            return ("❌ 无法获取当前位置：%s\n"
                    "若 /amcl_pose 没有数据，通常是雷达 /scan 无帧导致"
                    "AMCL 无法定位" % (reason or '未知原因'))

        x, y, yaw = float(pos['x']), float(pos['y']), float(pos['yaw'])
    except Exception as e:              # noqa: BLE001
        return f"❌ 获取当前位置失败: {e}"

    store = get_location_store()
    return store.save(name, x, y, yaw)


@tool(args_schema=NavigateToSavedInput)
def navigate_to_saved_location(name: str, return_home: bool = False) -> str:
    """
    导航到之前保存的自定义地点。

    当用户说"去厕所"、"去书房"时使用此工具。
    如果名称不是自定义地点，会回退尝试预定义地点。

    return_home 默认为 False —— **只去，不自动回起点**。
    ★ 2026-11-01 实机反馈：原来硬编码 return_to_start=True，用户只说
      “去厕所”，到了之后车又自己跑回起点，完全不是他要的。
      现在改成默认不回，只有用户明说“去了再回来”才设 True。
    """
    from autopatrol_robot.nav_manager import launch_navigate_with_return_async

    store = get_location_store()

    # 先查自定义地点
    index = store.find_index(name)
    if index is not None:
        return launch_navigate_with_return_async(
            index, name, return_to_start=bool(return_home))

    # 回退：查预定义地点
    if name in LOCATION_MAP:
        return launch_navigate_with_return_async(
            LOCATION_MAP[name], name, return_to_start=bool(return_home))

    # 都找不到
    custom = store.list_all()
    hint = "、".join(list(custom.keys())[:5]) if custom else "暂无"
    return (
        f"❌ 未找到地点：{name}。\n"
        f"预定义地点：{', '.join(LOCATION_MAP.keys())}\n"
        f"自定义地点：{hint}\n"
        f"请先用'这里是{name}'来标记位置。"
    )


@tool
def list_saved_locations() -> str:
    """
    列出所有已保存的自定义地点。

    当用户问"我保存了哪些地点"、"有哪些自定义地点"时使用。
    """
    store = get_location_store()
    custom = store.list_all()
    if not custom:
        return "📌 暂无自定义地点。你可以先移动到某个位置，然后说'这里是XXX'来标记。"

    result = "📌 已保存的自定义地点：\n"
    for name, coords in custom.items():
        result += f"  • {name}: ({coords[0]:.2f}, {coords[1]:.2f})\n"
    return result


@tool(args_schema=DeleteSavedLocationInput)
def delete_saved_location(name: str) -> str:
    """
    删除之前保存的自定义地点。

    当用户说"删除厕所"、"忘记这个位置"、"清除书房"时使用此工具。
    """
    store = get_location_store()
    return store.delete(name)


@tool
def clear_all_saved_locations() -> str:
    """
    删除所有已保存的自定义地点（不影响预定义的前 4 个航点）。

    当用户说"清空所有地点"、"删除所有标记"时使用此工具。
    """
    store = get_location_store()
    return store.clear_all()
