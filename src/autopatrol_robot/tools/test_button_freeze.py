#!/usr/bin/env python3
"""test_button_freeze.py
============================================================
「多点导航按钮确认 + 位姿冻结」需求的回归测试（2026-11-01）

【需求原文】
    单点导航完成后无需强制阻塞等待按按钮确认，只有两个点或以上的
    需要按按钮确认。比如我向 ai 助手下达：先去书房、再去客厅。此时
    小车到达书房后需阻塞等待按钮被按下而后前往下一航点，若等待超时
    则直接取消整个任务，但全程小车位姿要保持稳定。

【为什么位姿会飘】
    ESP32 的 esp32_command 在等按钮时**阻塞 micro-ROS 任务最多 60 秒**，
    期间 /odom 和 /imu_raw 全部停发。EKF 收不到观测就按上一帧速度
    继续积分 —— 实测 55 秒把 odom→base_footprint 从 0.2m 推到 20.6m
    （方向固定 53°，速度恒定 0.37 m/s）。

【对策】
    odom2tf 新增 /odom_freeze：冻结期间锁住位姿、只刷新时间戳；
    patrol_node 在调 esp32_command 之前置 true，之后（finally）置 false。

本测试不需要 ROS、不接触硬件。
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.join(HERE, '..', '..', '..')
PKG = os.path.join(WS, 'src', 'autopatrol_robot', 'autopatrol_robot')
ODOM2TF = os.path.join(WS, 'src', 'fishbot_bringup', 'src', 'odom2tf.cpp')
PATROL = os.path.join(PKG, 'patrol_node.py')
NAV_MGR = os.path.join(PKG, 'nav_manager.py')
NAV_TOOLS = os.path.join(PKG, 'tools', 'nav_tools.py')
LOC_TOOLS = os.path.join(PKG, 'tools', 'loc_tools.py')
WEB = os.path.join(PKG, 'web_controller.py')

ok = 0
total = 0


def check(name, cond, extra=''):
    global ok, total
    total += 1
    ok += bool(cond)
    print('  %s %s %s' % ('✅' if cond else '❌', name, extra))


def read(p):
    with open(p, encoding='utf-8') as f:
        return f.read()


# ============================================================
print('一、odom2tf 位姿冻结')
# ============================================================
cxx = read(ODOM2TF)

check('订阅 /odom_freeze', '"/odom_freeze"' in cxx)
check('有冻结开关参数 enable_freeze', 'enable_freeze' in cxx)
check('冻结状态变量 frozen_', 'frozen_' in cxx)
check('冻结时锁住 x/y/yaw 三个量',
      'frozen_x_' in cxx and 'frozen_y_' in cxx and 'frozen_yaw_' in cxx)
check('冻结期间忽略输入（不更新 last_raw）',
      re.search(r'if \(frozen_\) \{\s*\n\s*return;', cxx) is not None)
check('冻结期间用定时器自己重发（输入是断的）',
      'freeze_timer_' in cxx and 'freeze_tick_' in cxx)
check('重发频率 20Hz（nav2 要求 TF 200ms 内新鲜）',
      'std::chrono::milliseconds(50)' in cxx)
check('冻结/解冻都打日志',
      '位姿冻结开始' in cxx and '位姿解冻' in cxx)
check('解冻时**不**立刻算补偿量（否则会跳 24m）',
      'set_offset_from_(last_raw_x_' not in cxx)
check('解冻后等下一帧输入再重定位（pending_rebase_）',
      'pending_rebase_' in cxx)
check('重定位用**当前** raw 反解（不是旧的）',
      re.search(r'pending_rebase_ = false;\s*\n\s*set_offset_from_\(raw_x,\s*raw_y,\s*raw_yaw',
                cxx) is not None)
check('时间戳强制单调（TF buffer 不被污染）',
      'if (stamp < last_stamp_)' in cxx)
check('CMakeLists 加了 std_msgs 依赖',
      'find_package(std_msgs REQUIRED)' in
      read(os.path.join(WS, 'src', 'fishbot_bringup', 'CMakeLists.txt')))

# ---- 逻辑镜像：冻结行为的语义 ----
print()
print('  逻辑镜像（冻结语义）：')


class FakeOdom2TF:
    """镜像 odom2tf 的冻结 + 补偿逻辑。"""

    def __init__(self):
        self.frozen = False
        self.pending_rebase = False   # 解冻后等下一帧再重算补偿
        self.fx = self.fy = self.fyaw = 0.0
        self.has_last = False
        self.ox = self.oy = self.oyaw = 0.0   # 输出
        self.rx = self.ry = self.ryaw = 0.0   # 原始
        self.off_x = self.off_y = self.off_yaw = 0.0
        self.published = []

    def _apply(self, rx, ry, ryaw):
        import math
        c, s = math.cos(self.off_yaw), math.sin(self.off_yaw)
        return (self.off_x + c * rx - s * ry,
                self.off_y + s * rx + c * ry,
                ryaw + self.off_yaw)

    def _set_off(self, rx, ry, ryaw, wx, wy, wyaw):
        import math
        self.off_yaw = wyaw - ryaw
        c, s = math.cos(self.off_yaw), math.sin(self.off_yaw)
        self.off_x = wx - (c * rx - s * ry)
        self.off_y = wy - (s * rx + c * ry)

    def feed(self, rx, ry, ryaw, now):
        if self.frozen:
            self.published.append((self.fx, self.fy))   # 定时器重发
            return
        if self.pending_rebase:
            # 用**当前** raw 反解补偿量，使输出正好等于冻结值
            self.pending_rebase = False
            self._set_off(rx, ry, ryaw, self.fx, self.fy, self.fyaw)
        ox, oy, oyaw = self._apply(rx, ry, ryaw)
        self.rx, self.ry, self.ryaw = rx, ry, ryaw
        self.ox, self.oy, self.oyaw = ox, oy, oyaw
        self.has_last = True
        self.published.append((ox, oy))

    def freeze(self, on):
        if on and not self.frozen:
            self.frozen = True
            if self.has_last:
                self.fx, self.fy, self.fyaw = self.ox, self.oy, self.oyaw
        elif not on and self.frozen:
            self.frozen = False
            # ★ 不在这里算补偿量：此刻还没有新数据，用旧 raw 算出来
            #   的补偿量到下一帧就失效了（实测会跳 24m）。
            self.pending_rebase = True


d = FakeOdom2TF()
d.feed(0.0, 0.0, 0.0, 0.0)
d.freeze(True)
check('冻结后输出停在快照',
      d.fx == 0.0 and d.fy == 0.0)

# 模拟 EKF 盲推：这 60 秒里上游漂到 20 米
for k in range(60):
    d.feed(-0.4 * k, -0.5 * k, 0.0, k)
check('上游漂 20 米，冻结期间输出**始终**在原点',
      all(abs(x) < 1e-9 and abs(y) < 1e-9 for x, y in d.published[-60:]),
      '最后输出 %s' % (d.published[-1],))

d.freeze(False)
d.feed(-24.0, -30.0, 0.0, 61)
ox, oy = d.published[-1]
check('解冻后输出**仍**从冻结值继续（没有跳回落后的上游）',
      abs(ox) < 0.5 and abs(oy) < 0.5, '(%.2f, %.2f)' % (ox, oy))

# 之后正常移动，相对位移要正确
d.feed(-24.0 + 1.0, -30.0, 0.0, 62)
ox2, oy2 = d.published[-1]
moved = ((ox2 - ox) ** 2 + (oy2 - oy) ** 2) ** 0.5
check('解冻后相对位移正确（走 1m 就显示 1m）',
      abs(moved - 1.0) < 1e-6, '%.3fm' % moved)

# 朝向也要对
check('解冻后朝向补偿正确',
      abs(d.oyaw - 0.0) < 1e-9, '%.3f' % d.oyaw)

# ============================================================
print()
print('二、patrol_node：单点 vs 多点')
# ============================================================
pat = read(PATROL)

check('新增 set_odom_freeze', 'def set_odom_freeze(' in pat)
check('发布 /odom_freeze', "'/odom_freeze'" in pat)
check('连发 3 次（切换丢一帧就白等 60 秒）',
      'for _ in range(3):' in pat)
check('nav_to_pose 有 confirm_button 参数',
      'def nav_to_pose(self, target_pose, confirm_button: bool = False)'
      in pat)
check('新增 _wait_button_confirm', 'def _wait_button_confirm(' in pat)
check('★ 先冻结再调 esp32_command（顺序不能反）',
      pat.find('self.set_odom_freeze(True)')
      < pat.find('servo.call_servo_and_wait('))
check('★ finally 里一定解冻（异常也不会留下冻住的 TF）',
      re.search(r'finally:\s*\n\s*#[^\n]*\n\s*self\.set_odom_freeze\(False\)',
                pat) is not None
      or re.search(r'finally:\s*\n[^\n]*\n\s*self\.set_odom_freeze\(False\)',
                   pat) is not None)
check('按钮超时 → 返回 False（让上层取消整个任务）',
      'return False' in pat)
check('超时日志说明「取消整个任务」',
      '取消整个导航任务' in pat)
check('超时时间默认 60s（与 ESP32 固件一致）',
      re.search(r"declare_parameter\(\s*'servo_timeout_sec'\s*,\s*60", pat)
      is not None)
check('已删除 enable_servo_announce（改由「是否多点」决定）',
      'enable_servo_announce' not in pat)

print()
print('  逻辑镜像（main 的 confirm 决策）：')


def decide(n_points):
    """镜像 main() 里的判定：中间点等按钮，最后一个不等。"""
    return [idx < n_points - 1 for idx in range(n_points)]


check('单点 → 全程不等按钮', decide(1) == [False])
check('2 点 → [等, 不等]', decide(2) == [True, False])
check('3 点 → [等, 等, 不等]', decide(3) == [True, True, False])
check('5 点 → 只有最后一个不等',
      decide(5) == [True, True, True, True, False])

# ============================================================
print()
print('三、返回起点流程已彻底删除')
# ============================================================
nm = read(NAV_MGR)
nav = read(NAV_TOOLS)
loc = read(LOC_TOOLS)
web = read(WEB)

check('nav_manager 无 _async_navigate_and_return_worker 定义',
      'def _async_navigate_and_return_worker' not in nm)
check('nav_manager 无 _async_sequence_and_return_worker 定义',
      'def _async_sequence_and_return_worker' not in nm)
check('nav_manager 不再有 return_to_start 参数',
      'return_to_start: bool' not in nm)
check('launch_navigate_with_return_async 不自动返航',
      'launch_return_to_start(silent=True)' not in nm)
check('工具层无 return_home 参数',
      'return_home' not in nav and 'return_home' not in loc)
check('web_controller 不再播报 returning/completed 阶段',
      "announce_phases = {'arrived', 'failed'}" in web)

# ============================================================
print()
print('四、多点导航支持自定义地点')
# ============================================================
check('navigate_sequence 查 LocationStore',
      'get_location_store' in nav and 'store.find_index(loc)' in nav)
check('找不到时同时列出预定义 + 自定义地点',
      '预定义地点' in nav and '自定义地点' in nav)

print()
print('  逻辑镜像（地点名解析）：')


def resolve(name, preset, custom):
    """镜像 navigate_sequence 的名字→索引解析。"""
    if name in preset:
        return preset[name]
    return custom.get(name)


PRESET = {'起点': 1, '厨房': 2, '客厅': 3, '影音房': 4}
CUSTOM = {'书房': 5, '厕所': 6}

check('「书房」(自定义) → 索引 5', resolve('书房', PRESET, CUSTOM) == 5)
check('「客厅」(预定义) → 索引 3', resolve('客厅', PRESET, CUSTOM) == 3)
check('「火星」(不存在) → None', resolve('火星', PRESET, CUSTOM) is None)

print()
print('=' * 60)
print('通过 %d / %d' % (ok, total))
print('=' * 60)
sys.exit(0 if ok == total else 1)
