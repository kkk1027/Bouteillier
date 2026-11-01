#!/usr/bin/env python3
"""test_odom_reset_and_autoreturn.py
============================================================
两项实机事故的回归测试（2026-11-01）：

【事故 A：位姿缓慢漂移，只在「去厕所」后出现】
    实测 trace（odom→base_footprint）：
        12:24:12  ( -0.14,  -0.11)   |odom| = 0.2m
        12:24:22  ( -0.81,  -0.97)   |odom| = 1.3m
        12:24:45  ( -2.93,  -3.73)   |odom| = 4.7m
        12:25:07  (-12.20, -16.23)   |odom| = 20.3m   ← 55 秒漂了 20 米
        12:25:08  (  0.047,  0.039)  |odom| = 0.05m   ← 一帧归零

    根因两层：
      1. 舵机播报阻塞 60 秒（日志「⏰ 等待 ESP32 按钮按下超时」），
         期间 patrol_node 不处理回调、ESP32 停发 /odom 和 /imu_raw，
         EKF 只能按上一帧速度盲推 → 位姿沿固定方向匀速漂移。
      2. ESP32 的 odom 是纯累加无持久化，芯片一重启就归零，
         odom→base_footprint 在 TF 树里是唯一的里程计来源，一归零
         整棵树就崩。

    ★ 为什么「只有去厕所会飘、回起点不飘」：
        去厕所 → patrol_node → 有舵机播报
        回起点 → nav_to_pose  → 没有舵机播报

【事故 B：只说「去厕所」，到了之后车又自己跑回起点】
    工具里硬编码 return_to_start=True，用户根本没让它回来。

本测试不需要 ROS、不需要雷达、不接触硬件。
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.join(HERE, '..', '..', '..')
PKG = os.path.join(WS, 'src', 'autopatrol_robot', 'autopatrol_robot')
NAV_TOOLS = os.path.join(PKG, 'tools', 'nav_tools.py')
LOC_TOOLS = os.path.join(PKG, 'tools', 'loc_tools.py')
PATROL = os.path.join(PKG, 'patrol_node.py')
AGENT = os.path.join(PKG, 'agent_graph.py')
ODOM2TF = os.path.join(WS, 'src', 'fishbot_bringup', 'src', 'odom2tf.cpp')

ok = 0
total = 0


def check(name, cond, extra=''):
    global ok, total
    total += 1
    ok += bool(cond)
    print('  %s %s %s' % ('✅' if cond else '❌', name, extra))


def read(path):
    with open(path, encoding='utf-8') as f:
        return f.read()


# ============================================================
print('一、事故 A：odom 重置保护（odom2tf）')
# ============================================================
cxx = read(ODOM2TF)

check('包含速度门限参数 reset_speed_thresh',
      'reset_speed_thresh' in cxx)
check('包含角速度门限参数 reset_yawrate_thresh',
      'reset_yawrate_thresh' in cxx)
check('包含绝对跳变兜底 reset_jump_abs',
      'reset_jump_abs' in cxx)
check('有补偿状态 offset_x/y/yaw',
      'offset_x_' in cxx and 'offset_y_' in cxx and 'offset_yaw_' in cxx)
check('补偿做了绕 yaw 的旋转（不能直接加）',
      'std::cos(offset_yaw_)' in cxx and 'std::sin(offset_yaw_)' in cxx)
check('检测到重置时让输出保持上一帧',
      'last_out_x_' in cxx and 'set_offset_from_' in cxx)
check('重置时打告警日志（便于排查）',
      '里程计重置' in cxx)
check('时间戳强制单调（防 TF 缓存被污染）',
      'use_wall_clock' in cxx and 'last_stamp_' in cxx)
check('可以整体关闭保护（调试用）',
      'enable_reset_guard' in cxx)
check('不再依赖 tf2::getYaw（避免缺 tf2_geometry_msgs 链接）',
      # 注释里会提到这个名字，所以只检查「真正的调用」——
      # 即不以 * 或 // 开头的行里出现 tf2::getYaw
      not any('tf2::getYaw' in ln
              for ln in cxx.splitlines()
              if not ln.strip().startswith(('*', '//')))
      and 'yaw_from_quaternion' in cxx)
check('仍有 frame_id=odom / child=base_footprint',
      '"odom"' in cxx and '"base_footprint"' in cxx)

# 逻辑镜像：模拟数据里那段 20.6m 的归零
print()
print('  逻辑镜像（用 trace 里的真实数字）：')


def guard(prev, cur, dt, speed_thresh=1.0, yaw_thresh=3.15, jump_abs=1.0):
    """镜像 odom2tf 的判定。"""
    dist = ((cur[0] - prev[0]) ** 2 + (cur[1] - prev[1]) ** 2) ** 0.5
    return (dist / dt > speed_thresh) or (dist > jump_abs)


check('正常行驶（0.1m / 0.05s = 2m/s 会误报，0.1m/0.5s = 0.2m/s 正常）',
      guard((0.0, 0.0), (0.1, 0.0), 0.5) is False)
check('★ trace 里那次归零（20.6m → 0.05m，dt=0.3s）被识别',
      guard((-12.39, -16.50), (0.047, 0.039), 0.3) is True)
check('小跳变（2cm）不误报',
      guard((1.0, 1.0), (1.02, 1.0), 0.033) is False)
check('中速正常移动（0.3m/s，正好是本车上限）不误报',
      guard((0.0, 0.0), (0.03, 0.0), 0.1) is False)

# ============================================================
print()
print('二、事故 A 之二：舵机播报不再阻塞 60 秒')
# ============================================================
pat = read(PATROL)

check('新增 enable_servo_announce 参数', 'enable_servo_announce' in pat)
check('★ 默认关闭（避免 60 秒阻塞）',
      re.search(r"declare_parameter\(\s*'enable_servo_announce'\s*,\s*False",
                pat) is not None)
check('新增 servo_timeout_sec 参数（可调）',
      'servo_timeout_sec' in pat)
check('call_servo_and_wait 支持 heartbeat 回调',
      'heartbeat=None' in pat and 'heartbeat()' in pat)
check('等待循环里每 0.5s 调一次心跳（而不是一次 spin 到底）',
      'timeout_sec=0.5' in pat)
check('抽出了 _run_servo_announce，两处调用点统一',
      '_run_servo_announce' in pat)
check('注释里写清了事故根因（60s 阻塞 → odom 停发 → 盲推）',
      '盲推' in pat or '漂移' in pat)
check('没有残留的 call_servo_and_wait() 无参直调',
      'call_servo_and_wait()' not in pat)

# ============================================================
print()
print('三、事故 B：去X 不再自动回起点')
# ============================================================
nav = read(NAV_TOOLS)
loc = read(LOC_TOOLS)

check('nav_tools 的 NavigateToLocationInput 有 return_home',
      'return_home' in nav)
check('loc_tools 的 NavigateToSavedInput 有 return_home',
      'return_home' in loc)
check('★ navigate_to_saved_location 默认 return_home=False',
      'def navigate_to_saved_location(name: str, return_home: bool = False)'
      in loc)
check('★ navigate_to_location 默认 return_home=False',
      'def navigate_to_location(location: str, return_home: bool = False)'
      in nav)
check('不再有硬编码的 return_to_start=True（工具层）',
      'return_to_start=True)' not in nav and 'return_to_start=True)' not in loc)
check('序列导航也不自动返航',
      'return_to_start=False' in nav)
check('helper 改名后引用已同步（无 _navigate_to_and_return 残留）',
      '_navigate_to_and_return' not in nav)

# 逻辑镜像：工具到底传了什么
print()
print('  逻辑镜像（模拟工具调用）：')


def fake_launch(index, name, return_to_start=True):
    return {'index': index, 'name': name, 'return_to_start': return_to_start}


def navigate_to_saved(name, return_home=False):
    return fake_launch(1, name, return_to_start=bool(return_home))


check('「去厕所」→ return_to_start=False',
      navigate_to_saved('厕所')['return_to_start'] is False)
check('「去厕所」显式 return_home=True 才返航',
      navigate_to_saved('厕所', True)['return_to_start'] is True)

# ============================================================
print()
print('四、prompt / 文档同步')
# ============================================================
agent = read(AGENT)

check('prompt 不再说“内部已包含到达后自动回起点”',
      '内部已包含"到达后自动回起点"' not in agent)
check('prompt 说明了默认不回起点',
      '默认**只去、不回来**' in agent or '默认**不会**自动返回起点' in agent)
check('_EXCLUSIVE_NAV_TOOLS 注释已更新',
      '已**不再**默认自动回起点' in agent)

print()
print('=' * 60)
print('通过 %d / %d' % (ok, total))
print('=' * 60)
sys.exit(0 if ok == total else 1)
