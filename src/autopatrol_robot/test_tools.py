#!/usr/bin/env python3
"""
测试工具包装是否正确
"""

from autopatrol_robot.agent_tools import ALL_TOOLS

print("=" * 50)
print("已注册的 Agent 工具：")
print("=" * 50)

for tool in ALL_TOOLS:
    print(f"\n🔧 {tool.name}")
    print(f"   描述: {tool.description[:80]}...")
    print(f"   参数: {tool.args}")

# 测试调用工具（避免使用有问题的 get_current_position）
if __name__ == "__main__":
    from autopatrol_robot.agent_tools import (
        list_predefined_points,      # 列出预定义点
        get_battery_status,          # 获取电量（模拟）
        cleanup_ros
    )
    
    print("\n" + "=" * 50)
    print("测试1: 列出预定义目标点")
    print("=" * 50)
    try:
        result = list_predefined_points.invoke({})
        print(f"结果: {result}")
    except Exception as e:
        print(f"错误: {e}")
    
    print("\n" + "=" * 50)
    print("测试2: 获取电池状态")
    print("=" * 50)
    try:
        result = get_battery_status.invoke({})
        print(f"结果: {result}")
    except Exception as e:
        print(f"错误: {e}")
    
    print("\n" + "=" * 50)
    print("测试完成，跳过 get_current_position（TF 问题待修复）")
    print("=" * 50)
    
    # 清理
    cleanup_ros()