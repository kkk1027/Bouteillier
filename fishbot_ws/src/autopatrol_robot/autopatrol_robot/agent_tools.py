"""
agent_tools.py
===================
向后兼容的薄包装层 — 所有实际逻辑已迁移到 tools/ 子模块。
本文件保留 ALL_TOOLS 和 cleanup_ros() 导出，其他模块（agent_graph.py）的导入无需修改。

架构：
  agent_tools.py  ──import──>  tools/__init__.py  ──import──>  4 个工具模块
                                             ──import──>  location_store.py
                                             ──import──>  nav_manager.py
"""

# ============================================================
# 从 tools/ 子模块重新导出全部内容
# ============================================================
from autopatrol_robot.tools import (
    ALL_TOOLS,
    cleanup_ros,
    debug_log,
)

# 保留旧 API：兼容直接 from agent_tools import cleanup_ros 的调用方
from autopatrol_robot.tools import cleanup_ros as _cleanup_ros

# 以下为历史兼容保留，新代码请直接 import tools/*
# （agent_graph.py 已改为 from autopatrol_robot.tools import ALL_TOOLS, cleanup_ros）
