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

    dist_home = (x * x + y * y) ** 0.5

    # ============================================================
    # ★ 2026-11-01 覆盖前保护：拒绝把已有地点覆盖到原点附近
    # ------------------------------------------------------------
    # 实测事故（agent_node 原始日志）：
    #   [0] Human: 记住这里是书房   → save ✓ 书房 = (1.90, 0.35)
    #   [4] Human: 回起点           → return_to_start ✓
    #   [8] Human: 去书房           → save ✗✗✗ 一条纯导航指令
    #   [9] 书房被改写成 (0.00, -0.04) —— 车当时停在起点附近
    #
    # agent_graph 里已经加了 _guard_save_intent 做代码层纠错，但那是
    # **上游**防线；工具本身也不能无脑覆盖。两条独立防线都失效时，
    # 这里兜住，至少数据不会被写坏。
    #
    # 只在**三个条件同时成立**时拒绝：
    #   ① 地点已存在（本次是覆盖，不是新增）
    #   ② 新坐标离原点 < 0.30m（起点就在原点，几乎不可能是真目的地）
    #   ③ 旧坐标离原点 >= 0.30m（原来是个真实地点，覆盖=纯损失）
    # 如果旧坐标本来也在原点附近，覆盖没有损失，放行。
    # ============================================================
    old = store.get_coords(name)
    old_home = None
    if old is not None:
        old_home = (float(old[0]) ** 2 + float(old[1]) ** 2) ** 0.5

    if old is not None and dist_home < 0.30 and old_home >= 0.30:
        return (
            f"⛔ 没有覆盖「{name}」\n"
            f"　车现在在 ({x:.2f}, {y:.2f})，离地图原点只有 "
            f"{dist_home * 100:.0f} 厘米，\n"
            f"　而「{name}」原本记在 ({old[0]:.2f}, {old[1]:.2f})。\n"
            f"　这通常意味着车还没开到「{name}」。请先把车开过去，"
            f"再说一次「这里是{name}」。\n"
            f"　（如果确实要把它改记在原点附近，先删除该地点再重新保存。）"
        )

    msg = store.save(name, x, y, yaw)

    # ★ 2026-11-01 录点合理性检查
    #   实测事故：用户把「厕所」录在了 (0.056, -0.002)，而起点是 (0, 0)
    #   —— 两者相距 5.6 厘米。之后说"去厕所"，nav2 一收到目标就判定
    #   "已到达"（xy_goal_tolerance=0.25m），车一步没动。
    #   在录点的这一刻提醒，比等到导航时才发现有用得多。
    if dist_home < 0.30:
        msg += (
            f"\n⚠️ 注意：「{name}」的位置离地图原点只有 {dist_home * 100:.0f} 厘米"
            f"（起点也在原点附近）。\n"
            f"　如果这不是你想要的，请把车开到真正的「{name}」，"
            f"再说一次「这里是{name}」覆盖掉。\n"
            f"　否则以后让它去「{name}」时会原地不动、直接报「已到达」。"
        )
    return msg


@tool(args_schema=NavigateToSavedInput)
def navigate_to_saved_location(name: str) -> str:
    """
    导航到之前保存的自定义地点。

    当用户说"去厕所"、"去书房"时使用此工具。
    如果名称不是自定义地点，会回退尝试预定义地点。

    ★ 2026-11-01 用户确认：**只去，不自动回起点**。
      原来硬编码 return_to_start=True，用户只说“去厕所”，到了之后
      车又自己跑回起点 —— 完全不是他要的。这条流程已彻底删除。
    """
    from autopatrol_robot.nav_manager import launch_navigate_with_return_async

    store = get_location_store()

    # 先查自定义地点
    index = store.find_index(name)
    if index is not None:
        return launch_navigate_with_return_async(index, name)

    # 回退：查预定义地点
    if name in LOCATION_MAP:
        return launch_navigate_with_return_async(LOCATION_MAP[name], name)

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
