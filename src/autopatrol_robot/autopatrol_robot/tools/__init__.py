"""
tools/__init__.py
将所有工具汇总为 ALL_TOOLS 列表，供 agent_graph.py 使用。
"""

from autopatrol_robot.tools.nav_tools import (
    navigate_to_location,
    navigate_to_predefined_point,
    navigate_to_xy,
    navigate_sequence,
    cancel_navigation,
    return_to_start,
    get_navigation_status,
)
from autopatrol_robot.tools.loc_tools import (
    list_locations,
    save_current_location,
    navigate_to_saved_location,
    list_saved_locations,
    delete_saved_location,
    clear_all_saved_locations,
)
from autopatrol_robot.tools.follow_tools import (
    start_patrol_mode,
    stop_patrol_mode,
    get_patrol_mode_status,
)
from autopatrol_robot.tools.sys_tools import (
    get_current_position,
    initialize_pose,
    get_battery_status,
    list_predefined_points,
)
from autopatrol_robot.tools.butler_tools import (
    set_butler_mood,
    butler_say,
    randomize_butler_idle,
)
from autopatrol_robot.tools.web_tools import (
    get_weather,
    get_current_datetime,
)
from autopatrol_robot.tools.lidar_follow_tools import (
    start_lidar_follow,
    stop_lidar_follow,
    get_lidar_follow_status,
)

# 导出清理函数
from autopatrol_robot.tools.follow_tools import stop_follow_mode_process


def cleanup_ros():
    """清理 ROS2 资源（程序退出时调用）"""
    # 跟随类进程必须先停：它们会持续往 /cmd_vel 发指令，
    # 程序都退出了车还在跑是最危险的情况。
    print("[清理] 正在停止雷达跟随...")
    try:
        from autopatrol_robot.tools.lidar_follow_tools import (
            stop_lidar_follow_process)
        stop_lidar_follow_process(force=True)
    except Exception as e:                          # noqa: BLE001
        print("[清理] 停止雷达跟随失败: %s" % e)

    print("[清理] 正在停止看护模式...")
    stop_follow_mode_process(force=True)

    from autopatrol_robot.navigation_service import (
        get_global_nav_service,
    )
    service = get_global_nav_service()
    if service is not None:
        service.shutdown()
    
    import rclpy
    if rclpy.ok():
        rclpy.shutdown()


def debug_log(msg):
    """将调试信息写入文件"""
    import datetime
    with open('/tmp/agent_debug.log', 'a') as f:
        timestamp = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        f.write(f"[{timestamp}] {msg}\n")
        f.flush()


ALL_TOOLS = [
    # 导航工具
    navigate_to_xy,
    navigate_to_location,
    navigate_to_predefined_point,
    navigate_sequence,
    cancel_navigation,
    return_to_start,
    get_navigation_status,
    # 地点管理工具
    list_locations,
    save_current_location,
    navigate_to_saved_location,
    list_saved_locations,
    delete_saved_location,
    clear_all_saved_locations,
    # 看护模式工具
    start_patrol_mode,
    stop_patrol_mode,
    get_patrol_mode_status,
    # 系统状态工具
    get_current_position,
    initialize_pose,
    get_battery_status,
    list_predefined_points,
    # 角色表现工具（纯前端视觉，不控制硬件）
    set_butler_mood,
    butler_say,
    randomize_butler_idle,
    # 联网 / 实时信息（不控制硬件）
    get_weather,
    get_current_datetime,
    # 雷达跟随（会控制底盘，与导航/看护互斥）
    start_lidar_follow,
    stop_lidar_follow,
    get_lidar_follow_status,
]


# ============================================================
# 本地小模型专用工具子集
# ------------------------------------------------------------
# 3B 模型在 25 个工具（schema 9172 字符）下，工具选择准确率只有
# 12/14，且失败是**确定性**的（同一个输入每次都错，重试无效）。
# 砍到 18 个（schema 6534 字符）后回到 17/18。
#
# 去掉的 7 个都是「网页前端已有对应控件」或「口语里不会这么说」的：
#   navigate_to_location          —— navigate_to_saved_location 会自动回退
#   navigate_to_predefined_point  —— 按编号导航，没人这么说
#   navigate_to_xy               —— 网页有坐标输入控件
#   list_saved_locations         —— list_locations 已包含
#   clear_all_saved_locations    —— 危险操作，只保留在网页按钮上
#   list_predefined_points       —— 网页有预定义点列表
#
# 注意：initialize_pose **保留**。最初想砍掉（网页有按钮），
# 但用户口头说"初始化一下位姿"很常见，且 AMCL 失锁后这是恢复手段。
#
# 云端大模型不受此限制，仍然用完整的 ALL_TOOLS。
# ============================================================
_LOCAL_TOOL_NAMES = {
    "navigate_to_saved_location",
    "navigate_sequence",
    "return_to_start",
    "cancel_navigation",
    "get_navigation_status",
    "list_locations",
    "save_current_location",
    "delete_saved_location",
    "start_patrol_mode",
    "stop_patrol_mode",
    "get_patrol_mode_status",
    "get_current_position",
    "get_battery_status",
    # 用户真的会说"初始化一下位姿"，而且 AMCL 定位失效时这是恢复手段，
    # 不能只留在网页按钮上。实测加到 19 个工具仍稳定。
    "initialize_pose",
    "set_butler_mood",
    "butler_say",
    "randomize_butler_idle",
    "get_weather",
    "get_current_datetime",
    # 雷达跟随：用户口语里很可能直接说"跟着我"，必须给本地模型
    "start_lidar_follow",
    "stop_lidar_follow",
    "get_lidar_follow_status",
}

LOCAL_TOOLS = [t for t in ALL_TOOLS if t.name in _LOCAL_TOOL_NAMES]


def get_tools_for(provider: str):
    """按模型来源返回工具列表：本地用子集，云端用全量。"""
    return LOCAL_TOOLS if provider == "local" else ALL_TOOLS
