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

# 导出清理函数
from autopatrol_robot.tools.follow_tools import stop_follow_mode_process


def cleanup_ros():
    """清理 ROS2 资源（程序退出时调用）"""
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
]
