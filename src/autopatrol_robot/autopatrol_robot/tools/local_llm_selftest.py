#!/usr/bin/env python3
"""本地模型工具调用自检。

只做「自然语言 → 选哪个工具」的纯推理测试，
不会导入 rclpy、不会启动任何节点、不会让机器人移动。
所有工具都被替换成空壳，因此没有任何副作用。

用法：
    python3 -m autopatrol_robot.tools.local_llm_selftest
    # 或
    ./scripts/local_llm.sh test

退出码：准确率 < 85% 时返回 1，便于脚本化判断。
"""
import os
import sys
import time

# 自检固定走本地模型，不受当前 shell 的 FISHBOT_LLM 影响
os.environ["FISHBOT_LLM"] = "local"

try:
    from autopatrol_robot.tools import ALL_TOOLS
    from autopatrol_robot.agent_graph import (
        llm_with_tools, ACTIVE_SYSTEM_PROMPT, ACTIVE_TOOLS,
        LLM_MODEL, LOCAL_BASE_URL, _nonce_prompt,
    )
    from langchain_core.messages import HumanMessage, SystemMessage
except ImportError as e:
    print("❌ 导入失败：%s" % e)
    print("   请在 autopatrol_robot 包目录的父级运行，或先 source 工作空间：")
    print("   source install/setup.bash")
    sys.exit(2)

# 把每个工具的执行体换成空壳：模型仍然会生成真实的 tool_calls，
# 但就算被调用也只会返回一句话，不会碰硬件。
for _t in ALL_TOOLS:
    _t.func = lambda *a, **kw: "[SELFTEST] 已拦截，未执行"

CASES = [
    ("去厨房", ["navigate_to_saved_location", "navigate_to_location"]),
    ("带我回起点", ["return_to_start"]),
    ("机器人在哪", ["get_current_position"]),
    ("取消导航", ["cancel_navigation"]),
    ("把这里记成厕所", ["save_current_location"]),
    ("看看有哪些地点", ["list_locations"]),
    ("开始巡检", ["start_patrol_mode"]),
    ("停止巡检", ["stop_patrol_mode"]),
    ("电量还有多少", ["get_battery_status"]),
    ("先去厨房再去客厅", ["navigate_sequence"]),
    ("删除厕所这个地点", ["delete_saved_location"]),
    ("初始化一下位姿", ["initialize_pose", "get_current_position"]),
    ("巡检开着吗", ["get_patrol_mode_status"]),
    ("开心一点", ["set_butler_mood"]),
    ("说句话吧", ["butler_say"]),
    ("换个待机动作", ["randomize_butler_idle"]),
    ("保存当前位置为书房", ["save_current_location"]),
    ("取消当前任务", ["cancel_navigation"]),
    ("回起点", ["return_to_start"]),
    ("导航状态如何", ["get_navigation_status"]),
    # 联网 / 实时信息
    ("今天天气怎么样", ["get_weather"]),
    ("北京明天会下雨吗", ["get_weather"]),
    ("现在几点了", ["get_current_datetime"]),
    ("今天几号星期几", ["get_current_datetime"]),
    # 雷达跟随
    ("开始雷达跟随", ["start_lidar_follow"]),
    ("用雷达跟着我走", ["start_lidar_follow"]),
    ("停止雷达跟随", ["stop_lidar_follow"]),
    ("雷达跟随开着吗", ["get_lidar_follow_status"]),
]


def main():
    print("=" * 58)
    print("本地模型工具调用自检")
    print("  地址  : %s" % LOCAL_BASE_URL)
    print("  模型  : %s" % LLM_MODEL)
    print("  工具数: %d（完整 %d）" % (len(ACTIVE_TOOLS), len(ALL_TOOLS)))
    print("  提示词: %d 字符" % len(ACTIVE_SYSTEM_PROMPT))
    print("=" * 58)

    ok = 0
    total_time = 0.0
    fails = []
    for text, want in CASES:
        t0 = time.time()
        try:
            r = llm_with_tools.invoke([
                SystemMessage(content=_nonce_prompt(ACTIVE_SYSTEM_PROMPT)),
                HumanMessage(content=text),
            ])
        except Exception as e:  # noqa: BLE001
            fails.append((text, want, ["异常:" + str(e)[:30]]))
            print("❌ %-18s 异常: %s" % (text, str(e)[:50]))
            continue
        dt = time.time() - t0
        total_time += dt
        names = [c["name"] for c in (r.tool_calls or [])]
        hit = any(w in names for w in want)
        ok += hit
        if hit:
            print("✅ %-18s %4.1fs  %s" % (text, dt, names))
        else:
            print("❌ %-18s %4.1fs  应为 %s，实得 %s" %
                  (text, dt, want, names or "无"))
            fails.append((text, want, names))

    n = len(CASES)
    pct = ok * 100.0 / n
    print("-" * 58)
    print("准确率: %d/%d = %.1f%%   平均 %.2fs/条" %
          (ok, n, pct, total_time / n))
    if fails:
        print("失败用例 %d 条（见上）" % len(fails))
    print("=" * 58)
    return 0 if pct >= 85 else 1


if __name__ == "__main__":
    sys.exit(main())
