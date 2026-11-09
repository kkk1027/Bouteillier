"""
tools/nav_tools.py
AI 可调用的导航工具（全部走 CLI，通过 NavManager）。
"""

from langchain_core.tools import tool
from pydantic import BaseModel, Field

from autopatrol_robot.nav_manager import (
    launch_navigate_by_index,
    launch_navigate_sequence,
    launch_navigate_xy,
    launch_return_to_start,
    launch_navigate_with_return_async,
    launch_navigate_sequence_async,
    cancel_navigation as _cancel_nav,
    get_nav_status,
)
from autopatrol_robot.location_store import get_location_store

# ============================================================
# 预定义地点 → 1-based 索引映射
# ============================================================
LOCATION_MAP = {
    "起点": 1,
    "厨房": 2,
    "客厅": 3,
    "影音房": 4,
}


# ============================================================
# 引导页区域标签 → 地图坐标
# ============================================================
def _lookup_region(location):
    """在当前地图的区域标签里找同名区域，返回 (x, y) 或 None。

    这是「初次建图引导」和「AI 语义导航」之间的桥：用户在引导页框出
    「客厅」并起名后，说「去客厅」就能直接导航过去，**AI 侧不用改任何
    prompt** —— 命名地点本来就是自然语言，模型本来就是这么理解的。

    数据来源：maps/<map>_regions.json，由 web_controller 的
    /guide/regions 写入，坐标已在后端算好（像素 → 世界坐标）。

    ★ 为什么这个比 LOCATION_MAP 优先：
      LOCATION_MAP 指向 patrol_config.yaml 的 target_points，而那些
      坐标是**相对某一张具体地图**的。用户重新建图后原点/分辨率
      都可能变，旧坐标全部失效，导过去就是撞墙。
      而区域标签是基于**当前这张地图**画的，一定对得上。

    ★ 为什么延迟 import：
      web_controller 在启动时 import 了 tools 包（ALL_TOOLS），
      如果在模块顶层 import web_controller 会形成循环导入。
      butler_tools.py 也是同样的处理方式。
    """
    if not location:
        return None
    try:
        from autopatrol_robot import guide_map as gm
        from autopatrol_robot import web_controller as wc
    except Exception:                     # noqa: BLE001
        return None

    try:
        mdir = wc._maps_dir(wc.WORKSPACE_DIR)
        data, _err = gm.load_regions(mdir, wc.DEFAULT_MAP_NAME)
    except Exception:                     # noqa: BLE001
        return None

    target = str(location).strip()
    for item in data.get('regions', []):
        if str(item.get('name', '')).strip() != target:
            continue
        world = item.get('world') or {}
        try:
            return float(world['x']), float(world['y'])
        except (KeyError, TypeError, ValueError):
            return None
    return None


def _region_names():
    """列出当前地图上所有区域名（给失败提示用）。"""
    try:
        from autopatrol_robot import guide_map as gm
        from autopatrol_robot import web_controller as wc
        mdir = wc._maps_dir(wc.WORKSPACE_DIR)
        data, _err = gm.load_regions(mdir, wc.DEFAULT_MAP_NAME)
        return [str(r.get('name')) for r in data.get('regions', [])
                if r.get('name')]
    except Exception:                     # noqa: BLE001
        return []


# ============================================================
# 参数模型
# ============================================================
class NavToPoseInput(BaseModel):
    """导航到坐标点"""
    x: float = Field(..., description="目标点 X 坐标（米）")
    y: float = Field(..., description="目标点 Y 坐标（米）")
    yaw: float = Field(default=0.0, description="目标点朝向（弧度），默认 0")


class NavToPredefinedPointInput(BaseModel):
    """导航到预定义点（通过序号）"""
    index: int = Field(..., description="预定义目标点的序号，1-based")


class NavigateToLocationInput(BaseModel):
    """导航到命名地点"""
    location: str = Field(..., description="地点名称，例如：厨房、客厅、影音房、起点等")


class NavigateSequenceInput(BaseModel):
    """依次导航到多个地点"""
    locations: list[str] = Field(
        ...,
        description="要依次导航的地点名称列表。例如 ['厨房', '客厅'] 表示先去厨房再去客厅。"
    )


# ============================================================
# 公共 helper：导航到预定义点（默认**不**自动回起点）
# ============================================================

def _navigate_to(index: int, name: str) -> str:
    """导航到预定义点（异步非阻塞版，**不自动返航**）。

    ★ 2026-11-01 用户确认：**彻底删除「导航完成后自动返回起点」流程**。
      原来这里硬编码 return_to_start=True，用户只说“去厕所”，
      到了之后车又自己跑回去 —— 完全不是他要的。
      现在车到了就停在那儿；要回起点请调 return_to_start 工具。

    回调阶段（由 web_controller 注册）：
      'started'   → "📍 正在前往 XXX..."
      'arrived'   → "✅ 已到达 XXX！"
      'failed'    → "❌ 导航失败：..."
    """
    return launch_navigate_with_return_async(index, name)


# ============================================================
# 工具定义
# ============================================================

@tool(args_schema=NavigateToLocationInput)
def navigate_to_location(location: str) -> str:
    """导航到命名地点。

    当用户说"去厨房"、"去客厅"、"去书房"时使用此工具。
    **只去，不自动回起点。**

    地点来源按优先级：
      1. 用户在引导页画并命名的区域（任意名字，基于当前地图，最可靠）
      2. 预定义地点（起点/厨房/客厅/影音房）
    """
    # ① 先查引导页画的区域标签 —— 基于当前地图，坐标一定有效
    region = _lookup_region(location)
    if region is not None:
        x, y = region
        return launch_navigate_xy(x, y, 0.0)

    # ② 再查硬编码的预定义地点
    index = LOCATION_MAP.get(location)
    if index is None:
        names = _region_names()
        hint = ("、".join(names[:8]) if names else "暂无（可在引导页框选区域并命名）")
        return (f"❌ 未找到地点: {location}。\n"
                f"已标注区域：{hint}\n"
                f"预定义地点：{', '.join(LOCATION_MAP.keys())}")
    return _navigate_to(index, location)


@tool(args_schema=NavToPredefinedPointInput)
def navigate_to_predefined_point(index: int) -> str:
    """
    导航到预先配置的目标点（1-based 索引），到达后自动返回起点。

    当用户说"去第一个点"、"去第2个航点"时使用此工具。
    """
    if index < 1 or index > 4:
        return f"❌ 索引 {index} 无效，有效范围是 1-4"
    return _navigate_to(index, f"点{index}")


@tool(args_schema=NavToPoseInput)
def navigate_to_xy(x: float, y: float, yaw: float = 0.0) -> str:
    """
    让机器人导航到指定的地图坐标点（不污染 patrol_config.yaml）。

    当用户说"去某个位置"、"移动到坐标"时使用此工具。
    """
    return launch_navigate_xy(x, y, yaw)


@tool
def cancel_navigation() -> str:
    """取消当前导航。用户说停下/取消时用。"""
    return _cancel_nav()


@tool(args_schema=NavigateSequenceInput)
def navigate_sequence(locations: list[str]) -> str:
    """依次导航到多个地点，用户说先去A再去B时用。"""
    if not locations:
        return "❌ 导航队列为空"

    # 构建索引列表
    #
    # ★ 2026-11-01：支持自定义地点。
    #   原来只查 LOCATION_MAP（起点/厨房/客厅/影音房），而用户说
    #   “先去书房再去客厅”时，“书房”通常是「这里是书房」存下来的
    #   自定义地点，直接就会失败。
    #   自定义地点在 patrol_config.yaml 的 target_points 里也有索引
    #   （LocationStore.find_index 返回 i//3+1），patrol_node 完全
    #   认得，缺的只是这里的名字解析。
    from autopatrol_robot.location_store import get_location_store
    store = get_location_store()

    indices = []
    for i, loc in enumerate(locations, 1):
        index = LOCATION_MAP.get(loc)
        if index is None:
            try:
                index = store.find_index(loc)
            except Exception:          # noqa: BLE001
                index = None
        if index is None:
            custom = store.list_all()
            hint = "、".join(list(custom.keys())[:6]) if custom else "暂无"
            return (f"❌ 第{i}步失败：未找到地点 '{loc}'。\n"
                    f"预定义地点：{', '.join(LOCATION_MAP.keys())}\n"
                    f"自定义地点：{hint}\n"
                    f"请先用'这里是{loc}'来标记位置。")
        indices.append(index)

    names = " → ".join(locations)
    # 序列导航（patrol_node）内部会依次走完所有航点：
    # 中间点到达后等按钮确认（期间位姿冻结），最后一点到达即结束。
    return launch_navigate_sequence_async(indices, names)


@tool
def return_to_start() -> str:
    """返回起点。用户说回家/回起点时用。"""
    return launch_return_to_start()


@tool
def get_navigation_status() -> str:
    """查询导航状态。用户问到了吗时用。"""
    return get_nav_status()
