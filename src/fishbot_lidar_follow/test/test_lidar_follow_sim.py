#!/usr/bin/env python3
"""
雷达跟随离线仿真测试 —— 用假雷达数据驱动真实控制逻辑。

这个测试的价值：把「目标在 X 米、Y 度」喂进去，直接检查算出来的
/cmd_vel 是否符合预期。不需要雷达、不需要底盘、不需要连小车。

安全性：
    * 完全不创建 ROS 节点（Node 的构造、publisher、timer 全部被 mock）
    * 不发布任何真实消息，cmd_vel 只写进内存里的 Recorder
    * 把 stop_nav_on_start 关掉，不会去 kill 任何进程
    * 锁文件重定向到 /tmp 下的测试文件

用法：
    source install/setup.bash
    python3 src/fishbot_lidar_follow/test/test_lidar_follow_sim.py
"""

import math
import os
import sys
import time
import types
import unittest.mock as mock

_here = os.path.dirname(os.path.abspath(__file__))
_pkg = os.path.dirname(_here)
if os.path.isdir(os.path.join(_pkg, 'fishbot_lidar_follow')):
    sys.path.insert(0, _pkg)

import rclpy.node  # noqa: E402

from fishbot_lidar_follow import geometry as GEO  # noqa: E402
from fishbot_lidar_follow import lidar_follow as LF  # noqa: E402

PASS, FAIL = 0, 0


def check(name, cond, detail=''):
    global PASS, FAIL
    if cond:
        PASS += 1
        print('  ✅ %s%s' % (name, ('  ' + detail) if detail else ''))
    else:
        FAIL += 1
        print('  ❌ %s%s' % (name, ('  ' + detail) if detail else ''))


# ============================================================
# 记录器：代替真实 publisher
# ============================================================
class Recorder:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)

    def last(self):
        return self.msgs[-1] if self.msgs else None


class FakeLogger:
    def __init__(self):
        self.lines = []

    def _log(self, lvl, msg, *a):
        self.lines.append('[%s] %s' % (lvl, msg % a if a else msg))

    def info(self, m, *a):
        self._log('INFO', m, *a)

    def warn(self, m, *a):
        self._log('WARN', m, *a)

    def error(self, m, *a):
        self._log('ERROR', m, *a)

    def debug(self, m, *a):
        pass


class FakeScan:
    """构造一圈扫描：ranges[i] 对应角度 angle_min + i*angle_increment。"""

    def __init__(self, objects=(), n=721, rmin=0.30, rmax=64.0, far=20.0):
        """
        objects: [(距离, 角度deg)] 或 [(距离, 角度deg, 半宽点数), ...]
                 默认半宽 2（即 ±2，共 5 个点，模拟有宽度的实体）。
                 人腿用半宽 0（每个腿只占 1 个点）—— 实机就是这个工况。
        """
        self.angle_min = math.radians(-180.0)
        self.angle_increment = math.radians(0.5)
        self.range_min = rmin
        self.range_max = rmax
        self.ranges = [far] * n

        for obj in objects:
            dist, deg = obj[0], obj[1]
            half = obj[2] if len(obj) > 2 else 2
            idx = int(round((math.radians(deg) - self.angle_min)
                            / self.angle_increment))
            for k in range(-half, half + 1):
                j = idx + k
                if 0 <= j < n:
                    self.ranges[j] = dist + abs(k) * 0.01


def legs_objects(depth, half_gap=0.11, yaw_deg=0.0, hide=None):
    """生成「两条腿」的扫描对象，每条腿只占 1 个点。

    depth     人体中心到车的纵向距离（米）
    half_gap  每条腿相对中心线的横向偏移（米）
    yaw_deg   人整体绕车转过的角度
    hide      None=两条腿都显示；0/1=只显示其中一条（模拟遮挡）
    """
    yaw = math.radians(yaw_deg)
    cy, sy = math.cos(yaw), math.sin(yaw)
    out = []
    for idx, sign in enumerate((-1.0, 1.0)):
        if hide is not None and idx != hide:
            continue
        xf = depth              # 纵向：+x 为车正前方
        yl = sign * half_gap    # 横向：+y 为车的左侧
        xr = xf * cy - yl * sy
        yr = xf * sy + yl * cy
        out.append((math.hypot(xr, yr),
                    math.degrees(math.atan2(yr, xr)),
                    0))
    return out


# 从节点自己的 DEFAULT_PARAMS 派生，避免两份清单不同步。
# 只覆盖测试需要的两处：
#   stop_nav_on_start=False  —— 测试里绝不杀进程
#   publish_debug=False      —— 不需要调试话题
DEFAULTS = dict(LF.DEFAULT_PARAMS)
DEFAULTS['stop_nav_on_start'] = False
DEFAULTS['publish_debug'] = False
# 锁定稳定观察期默认关掉：它按真实时钟计时，而测试是「瞬发」喂帧，
# 开着会把所有控制用例都拦在 HOLD。它本身由 [9] 单独验证。
DEFAULTS['lock_settle_time'] = 0.0

# 「喂够帧数让节点锁定」要喂多少帧。
# 不要硬编码：lock_confirm_frames 一改，硬编码的用例就会悄悄失效。
# +2 是留余量（第 1 帧只建立候选，不算命中）。
LOCK_N = int(DEFAULTS['lock_confirm_frames']) + 2


def build_node(**overrides):
    params = dict(DEFAULTS)
    params.update(overrides)

    cmd_rec = Recorder()
    state_rec = Recorder()
    logger = FakeLogger()

    def fake_node_init(self, name, **kw):
        pass

    patches = [
        mock.patch.object(rclpy.node.Node, '__init__', fake_node_init),
        mock.patch.object(rclpy.node.Node, 'declare_parameter',
                          lambda self, k, v: None),
        mock.patch.object(
            rclpy.node.Node, 'get_parameter',
            lambda self, k: types.SimpleNamespace(value=params[k])),
        mock.patch.object(rclpy.node.Node, 'create_publisher',
                          lambda self, *a, **kw: cmd_rec),
        mock.patch.object(rclpy.node.Node, 'create_subscription',
                          lambda self, *a, **kw: None),
        mock.patch.object(rclpy.node.Node, 'create_timer',
                          lambda self, *a, **kw: None),
        mock.patch.object(rclpy.node.Node, 'get_logger',
                          lambda self: logger),
    ]
    for p in patches:
        p.start()

    node = LF.LidarFollowNode()
    node.cmd_pub = cmd_rec
    node.state_pub = state_rec
    # 实例属性会覆盖类方法，这样 patch 撤销后依然有效
    node.get_logger = lambda: logger
    node.debug_pub = None

    # 还原 patch（节点已构造完成，后续方法调用不再依赖 Node 基类）
    for p in patches:
        p.stop()

    return node, cmd_rec, state_rec, logger


def feed(node, scan, times=1):
    """喂若干帧扫描，模拟真实雷达的 5Hz（dt=0.2s）。

    ★ 必须模拟时间间隔。节点用 time.monotonic() 作为时间戳，
    而测试循环跑得极快，dt 几乎为 0 —— 于是卡尔曼的 predict 步
    完全不起作用，只剩 update，滤波输出严重滞后于测量值
    （实测：测量 0.85m 时滤波值只走到 0.63m）。
    真实雷达是 5Hz，所以这里每帧把时间戳往回拨 0.2s。
    """
    for _ in range(times):
        if node.tracker.last_stamp > 0.0:
            node.tracker.last_stamp -= 0.2
        node.on_scan(scan)
        time.sleep(0.0)


def lock_target(node, dist=0.38, angle=0.0, times=LOCK_N):
    """先在小距离锁定目标。

    设计上首次锁定必须是：
      * 单簇目标在 lock_max_range(0.50m) 内
      * 或者双腿对两点各在 pair_lock_max_range(1.00m) 内
      * 且连续确认 lock_confirm_frames(3) 帧
    """
    for _ in range(times):
        feed(node, FakeScan([(dist, angle)]))
    return node.locked


def settle(node, dist, angle_deg=0.0, times=25):
    """让卡尔曼收敛到新的 (dist, angle)。

    ★ 必须渐变，不能瞬移。
    节点有 max_target_jump(0.30m) 保护：目标距离一次跳变超过 0.30m
    会被判为「换了个物体」，当作丢失处理。
    真实目标也不会一帧之内瞬移半米（5Hz 下 0.30m/帧 = 1.5m/s，
    比跑步还快），所以测试也要按物理规律渐变。

    参数：
        angle_deg  目标角度，**单位为度**（与 FakeScan 一致）。
                   内部会与 node.target_angle（弧度）统一后再插值。
    """
    target_a = math.radians(angle_deg)
    start_d = node.target_dist if node.locked else dist
    start_a = node.target_angle if node.locked else target_a

    for i in range(1, times + 1):
        f = i / float(times)
        d = start_d + (dist - start_d) * f
        a = start_a + (target_a - start_a) * f
        feed(node, FakeScan([(d, math.degrees(a))]))


def tick(node, cmd_rec, n=12, dt=0.05):
    """跑 n 个控制周期（默认 12×0.05=0.6s，让加速度限幅走完）。"""
    for _ in range(n):
        node.last_ctrl_time -= dt
        node.on_ctrl()
    return cmd_rec.last()


def reset(node):
    node.state = LF.ST_SEARCH
    node.locked = False
    node.tracker.reset()
    node.target_valid = False
    node.last_lin = 0.0
    node.last_ang = 0.0
    node.last_scan_time = 0.0
    node.last_clusters = []
    node.guard_clusters = []
    node.lock_hits = 0
    node.lock_miss = 0
    node.lock_cand = None
    node.lock_kind = ''


# ============================================================
# 场景测试
# ============================================================
def test_lock_rules():
    print('\n[1] 目标锁定规则')

    # 目标在 0.9m，超出 lock_max_range(0.40) → 不应锁定
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan([(0.90, 0.0)]))
    check('0.90m 目标超过首次锁定范围，不锁定', not node.locked,
          'locked=%s' % node.locked)

    # 目标在 0.35m → 应锁定（需连续 3 帧确认）
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan([(0.35, 0.0)]))
    check('单帧不锁定（时间确认生效）', not node.locked,
          'hits=%d' % node.lock_hits)
    feed(node, FakeScan([(0.35, 0.0)]), times=LOCK_N)
    check('0.35m 目标在 50cm 内，连续 %d 帧后锁定' % LOCK_N, node.locked,
          'locked=%s dist=%.2f' % (node.locked, node.target_dist))

    # 锁定后目标退到 1.0m，仍应继续跟踪（track_max_range=1.20）
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan([(0.35, 0.0)]), times=LOCK_N)
    for _ in range(20):
        feed(node, FakeScan([(1.00, 0.0)]))
    check('锁定后目标退到 1.00m 仍保持跟踪', node.locked and node.target_valid,
          'locked=%s valid=%s dist=%.2f'
          % (node.locked, node.target_valid, node.target_dist))

    # 目标跑到 1.5m（超过 track_max_range）→ 丢失
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan([(0.35, 0.0)]), times=LOCK_N)
    for _ in range(10):
        feed(node, FakeScan([(1.50, 0.0)]))
    check('目标超出跟踪范围后判定丢失', not node.target_valid,
          'valid=%s' % node.target_valid)

    # 搜索扇区外的目标不锁定。
    # 未锁定时的搜索扇区是 ±75°（lock_sector_half_angle_deg），
    # 所以用 +85° 来测「扇区外」。
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan([(0.35, 85.0)]), times=LOCK_N)
    check('±75° 搜索扇区外的目标不锁定', not node.locked,
          'locked=%s' % node.locked)

    # 但 ±75° 扇区内的侧面目标应该能锁定
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan([(0.35, 60.0)]), times=LOCK_N)
    check('+60° 侧面目标能被锁定（宽搜索扇区生效）', node.locked,
          'locked=%s dist=%.2f ang=%.0f°'
          % (node.locked, node.target_dist,
             math.degrees(node.target_angle)))


def test_approach_hold_retreat():
    print('\n[2] 前进 / 停 / 后退')

    # ---- 先锁定，再让目标退到 0.85m → 前进 ----
    node, rec, _, _ = build_node()
    reset(node)
    lock_target(node, 0.38)
    settle(node, 0.85)
    m = tick(node, rec)
    check('锁定后目标退到 0.85m → 前进',
          m is not None and m.linear.x > 0.0,
          '线速度=%.3f 状态=%s 距离=%.2f'
          % (m.linear.x if m else 0, node.state, node.target_dist))
    check('  前进速度不超过上限 %.2f' % 0.22,
          m is not None and m.linear.x <= 0.22 + 1e-6,
          '%.3f' % (m.linear.x if m else 0))

    # ---- 锁定后目标退到 0.60m（死区内）→ 停 ----
    node, rec, _, _ = build_node()
    reset(node)
    lock_target(node, 0.38)
    settle(node, 0.60)
    m = tick(node, rec, n=25)
    check('目标 0.60m 在死区内 → 停',
          m is not None and abs(m.linear.x) < 1e-6,
          '线速度=%.3f 状态=%s 距离=%.2f'
          % (m.linear.x if m else 0, node.state, node.target_dist))

    # ---- 目标 0.35m（锁定后就一直很近）→ 后退 ----
    node, rec, _, _ = build_node()
    reset(node)
    settle(node, 0.35)
    m = tick(node, rec, n=20)
    check('目标 0.35m → 后退', m is not None and m.linear.x < 0.0,
          '线速度=%.3f 状态=%s' % (m.linear.x if m else 0, node.state))
    check('  后退速度不超过 %.2f' % 0.12,
          m is not None and abs(m.linear.x) <= 0.12 + 1e-6,
          '%.3f' % abs(m.linear.x if m else 0))


def test_angle_control():
    print('\n[3] 角度对准')

    # ---- 目标在左 +20°，距离 0.85 → 先转向，不前进 ----
    node, rec, _, _ = build_node()
    reset(node)
    lock_target(node, 0.38, 0.0)
    settle(node, 0.85, 20.0)
    m = tick(node, rec)
    check('目标偏左 20° → 原地左转',
          m is not None and m.angular.z > 0.0,
          '角速度=%.3f' % (m.angular.z if m else 0))
    check('  大角度偏差时不前进', m is not None and abs(m.linear.x) < 1e-6,
          '线速度=%.3f' % (m.linear.x if m else 0))

    # ---- 目标在右 -20° → 右转 ----
    node, rec, _, _ = build_node()
    reset(node)
    lock_target(node, 0.38, 0.0)
    settle(node, 0.85, -20.0)
    m = tick(node, rec)
    check('目标偏右 20° → 原地右转',
          m is not None and m.angular.z < 0.0,
          '角速度=%.3f' % (m.angular.z if m else 0))

    # ---- 目标在 +1°，死区内 → 不转 ----
    node, rec, _, _ = build_node()
    reset(node)
    lock_target(node, 0.38, 0.0)
    settle(node, 0.85, 1.0)
    m = tick(node, rec)
    check('角度误差 1° 在死区内 → 不转向',
          m is not None and abs(m.angular.z) < 1e-6,
          '角速度=%.3f' % (m.angular.z if m else 0))

    # ---- 目标在 +20° 但很远(1.1m) → 允许边走边转 ----
    node, rec, _, _ = build_node()
    reset(node)
    lock_target(node, 0.38, 0.0)
    settle(node, 1.10, 20.0)
    m = tick(node, rec)
    check('目标偏 20° 但距离 1.1m → 允许边走边转',
          m is not None and m.linear.x > 0.0 and m.angular.z > 0.0,
          '线=%.3f 角=%.3f' % (m.linear.x if m else 0, m.angular.z if m else 0))


def test_safety():
    print('\n[4] 安全兜底')

    # ---- 没有扫描数据 → 停 ----
    node, rec, _, _ = build_node()
    reset(node)
    m = tick(node, rec)
    check('没有收到扫描 → 输出 0', m is not None and abs(m.linear.x) < 1e-6
          and abs(m.angular.z) < 1e-6, '线=%.3f 角=%.3f'
          % (m.linear.x if m else 0, m.angular.z if m else 0))

    # ---- 雷达超时（有过数据但断了很久）→ 停 ----
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan([(0.85, 0.0)]), times=LOCK_N)
    node.last_scan_time -= 2.0        # 假装 2 秒没数据
    m = tick(node, rec, n=3)
    check('雷达超时 → 输出 0', m is not None and abs(m.linear.x) < 1e-6,
          '线=%.3f' % (m.linear.x if m else 0))

    # ---- 目标丢失 → 停 ----
    node, rec, _, _ = build_node()
    reset(node)
    lock_target(node, 0.38)
    settle(node, 0.85, times=8)
    # 连续喂空场景（所有点都很远）
    for _ in range(10):
        feed(node, FakeScan([]))
    m = tick(node, rec, n=5)
    check('目标丢失 → 输出 0', m is not None and abs(m.linear.x) < 1e-6,
          '线=%.3f valid=%s' % (m.linear.x if m else 0, node.target_valid))

    # ---- 后退时后方有障碍 → 不许后退 ----
    node, rec, _, _ = build_node()
    reset(node)
    # 目标很近（想后退）+ 后方 0.30m 有障碍
    for _ in range(18):
        feed(node, FakeScan([(0.35, 0.0), (0.30, 180.0)]))
    m = tick(node, rec, n=20)
    check('后退时后方 0.30m 有障碍 → 不后退',
          m is not None and abs(m.linear.x) < 1e-6,
          '线=%.3f 状态=%s' % (m.linear.x if m else 0, node.state))

    # ---- 同样的目标，后方无障碍 → 正常后退 ----
    node, rec, _, _ = build_node()
    reset(node)
    for _ in range(18):
        feed(node, FakeScan([(0.35, 0.0)]))
    m = tick(node, rec, n=20)
    check('后方无障碍 → 正常后退',
          m is not None and m.linear.x < 0.0,
          '线=%.3f 状态=%s' % (m.linear.x if m else 0, node.state))

    # ---- 前进时前方突现更近的障碍 → 刹车 ----
    # 目标在 0.85m，另有一个 0.31m 的东西挡在 12° 方向
    node, rec, _, _ = build_node()
    reset(node)
    lock_target(node, 0.38, 0.0)
    settle(node, 0.85, 0.0)
    for _ in range(10):
        feed(node, FakeScan([(0.85, 0.0), (0.31, 12.0)]))
    m = tick(node, rec, n=10)
    check('前方 0.31m 突现障碍 → 不前进',
          m is not None and abs(m.linear.x) < 1e-6,
          '线=%.3f 状态=%s 距离=%.2f'
          % (m.linear.x if m else 0, node.state, node.target_dist))


def test_short_dropout():
    print('\n[5] 短暂丢帧容忍')

    node, rec, _, _ = build_node()
    reset(node)
    lock_target(node, 0.38)
    settle(node, 0.85, times=10)
    check('先正常锁定', node.locked and node.target_valid,
          'locked=%s valid=%s' % (node.locked, node.target_valid))

    # 丢 2 帧（< lost_threshold=3）→ 应保持 valid
    feed(node, FakeScan([]))
    feed(node, FakeScan([]))
    check('丢 2 帧仍认为目标存在（用预测值维持）', node.target_valid,
          'valid=%s lost_frames=%d' % (node.target_valid,
                                       node.tracker.lost_frames))

    # 再丢 2 帧（累计 4 > 3）→ 判定丢失
    feed(node, FakeScan([]))
    feed(node, FakeScan([]))
    check('连续丢帧超过阈值 → 判定丢失', not node.target_valid,
          'valid=%s lost_frames=%d' % (node.target_valid,
                                       node.tracker.lost_frames))


def test_smoothing():
    print('\n[6] 速度平滑（加速度限幅）')

    node, rec, _, _ = build_node()
    reset(node)
    lock_target(node, 0.38)
    settle(node, 1.15, times=10)

    # 第 1 个周期的输出应该远小于稳态值（因为从 0 开始加速）
    node.last_ctrl_time -= 0.05
    node.on_ctrl()
    first = rec.last().linear.x

    m = tick(node, rec, n=25)
    steady = m.linear.x

    check('起步不突跳（加速度限幅生效）',
          first < steady * 0.5 + 1e-3,
          '首周期=%.3f 稳态=%.3f' % (first, steady))
    check('首周期不超过 accel*dt',
          first <= 0.80 * 0.05 + 1e-6,
          '首周期=%.3f 上限=%.3f' % (first, 0.80 * 0.05))


def test_leg_pair_lock():
    print('\n[7] 双腿成对锁定（实机工况）')

    # ---- 基本：两条腿各 1 个点，0.62m，间距 22cm ----
    node, rec, _, _ = build_node()
    reset(node)
    legs = legs_objects(0.62)
    feed(node, FakeScan(legs))
    check('双腿单帧不锁定（要 %d 帧确认）' % LOCK_N, not node.locked,
          'hits=%d' % node.lock_hits)
    feed(node, FakeScan(legs), times=LOCK_N)
    check('双腿各 1 点也能锁定', node.locked,
          'locked=%s kind=%s' % (node.locked, node.lock_kind))
    check('  锁定类型识别为 pair', node.lock_kind == 'pair',
          node.lock_kind)
    check('  中点距离 ≈ 0.62m', abs(node.target_dist - 0.62) < 0.05,
          '%.3f' % node.target_dist)
    check('  中点角度 ≈ 0°', abs(math.degrees(node.target_angle)) < 3.0,
          '%+.1f°' % math.degrees(node.target_angle))

    # ---- 中点比单条腿更接近真实身体中心 ----
    # 单独一条腿在 0.63m/±10°，中点应该在 0.62m/0°
    check('  中点角度比单腿角度小（修掉了左右跳）',
          abs(node.target_angle) < math.radians(5.0),
          '单腿 ±10.1° → 中点 %+.1f°' % math.degrees(node.target_angle))

    # ---- 间距超过 60cm → 不算一双腿 ----
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan(legs_objects(0.62, half_gap=0.375)), times=LOCK_N)
    check('两点间距 0.75m 超过 60cm → 不锁定', not node.locked,
          'locked=%s' % node.locked)

    # ---- 深度差超过 35cm → 是「腿 + 墙」不是一双腿 ----
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan([(0.55, -8.0, 0), (0.95, 8.0, 0)]), times=LOCK_N)
    check('两点深度差 0.40m → 不锁定', not node.locked,
          'locked=%s' % node.locked)

    # ---- 锁定后一条腿被挡住 2 帧 → 卡尔曼顶住 ----
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan(legs_objects(0.62)), times=LOCK_N)
    ok = node.locked
    feed(node, FakeScan(legs_objects(0.62, hide=0)), times=2)
    check('一条腿被挡住 2 帧 → 仍保持跟随',
          ok and node.target_valid,
          'locked=%s valid=%s lost=%d'
          % (node.locked, node.target_valid, node.tracker.lost_frames))

    # ---- 两条腿整体转 15° → 角度跟着走 ----
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan(legs_objects(0.62)), times=LOCK_N)
    for i in range(1, 11):
        feed(node, FakeScan(legs_objects(0.62, yaw_deg=15.0 * i / 10.0)))
    check('双腿整体转 15° → 目标角度跟上',
          node.target_valid
          and abs(math.degrees(node.target_angle) - 15.0) < 4.0,
          '角度 %+.1f°' % math.degrees(node.target_angle))

    # ---- 人侧身站：两条腿合成一个 5 点大簇 → 走单簇路径，不能丢 ----
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan(legs_objects(0.62)), times=LOCK_N)
    kinds = [node.lock_kind]
    for _ in range(6):
        feed(node, FakeScan([(0.62, 0.0)]))     # 默认半宽 2 → 5 个点
        kinds.append('valid' if node.target_valid else 'lost')
    check('双腿合成单簇后依然跟随（模式切换不打嗝）',
          node.target_valid and 'lost' not in kinds,
          ' '.join(kinds))

    # ---- 噪声：每次位置都在跳的孤立点，永远不该锁定 ----
    node, rec, _, _ = build_node()
    reset(node)
    locked_any = False
    for i in range(12):
        seed = i * 37
        d1 = 0.45 + (seed % 9) * 0.05
        a1 = -20.0 + (seed % 7) * 8.0
        d2 = 0.45 + ((seed + 3) % 9) * 0.05
        a2 = -20.0 + ((seed + 5) % 7) * 8.0
        feed(node, FakeScan([(d1, a1, 0), (d2, a2, 0)]))
        if node.locked:
            locked_any = True
    check('跳动的噪声点不会被锁定', not locked_any,
          'locked=%s' % locked_any)

    # ---- 实体纸箱（点够多）仍走单簇路径 ----
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan([(0.42, 0.0)]), times=LOCK_N)
    check('实体目标（5 点单簇）仍按单簇锁定',
          node.locked and node.lock_kind == 'single',
          'kind=%s' % node.lock_kind)

    # ---- 双腿 + 更近的实体同时存在 → 锁更近的那个 ----
    node, rec, _, _ = build_node()
    reset(node)
    mixed = legs_objects(0.62) + [(0.40, 0.0, 2)]
    feed(node, FakeScan(mixed), times=LOCK_N)
    check('双腿与更近实体同时存在 → 锁更近的实体',
          node.locked and node.lock_kind == 'single'
          and abs(node.target_dist - 0.40) < 0.05,
          'kind=%s dist=%.2f' % (node.lock_kind, node.target_dist))

    # ---- 横向张得太开的一对不是双腿 ----
    # 实机仿真里出现过「间距 0.56m、角度差 49°」的假双腿：
    # 两点直线距离 0.56 < 60cm 上限，但横向 0.55m 远超人体宽度。
    node, rec, _, _ = build_node()
    reset(node)
    fake = [(0.61, 6.0, 0), (0.73, 55.0, 0)]
    feed(node, FakeScan(fake), times=LOCK_N)
    check('横向 0.55m 的「假双腿」不锁定（横向间距约束）',
          not node.locked, 'locked=%s' % node.locked)

    # ---- 腿距每帧都变 → 间距稳定性拦住 ----
    # 注意：腿距必须取「两侧都在允许范围内」的值（0.15 / 0.45），
    # 否则大间距那一帧会被横向约束整帧过滤掉，
    # 变成「漏检」而不是「间距不一致」，测不到这条约束。
    node, rec, _, _ = build_node()
    reset(node)
    seq = [0.15, 0.45, 0.18, 0.42, 0.15, 0.45, 0.18, 0.42]
    for g in seq:
        feed(node, FakeScan(legs_objects(0.62, half_gap=g / 2.0)))
    check('腿距 0.15↔0.45 交替 → 不锁定（间距稳定性约束）',
          not node.locked,
          'locked=%s sep 序列 %s' % (node.locked, seq))

    # ---- 腿距稳定（人正常走）→ 照常锁定 ----
    node, rec, _, _ = build_node()
    reset(node)
    for g in (0.21, 0.23, 0.22, 0.24, 0.22):
        feed(node, FakeScan(legs_objects(0.62, half_gap=g / 2.0)))
    check('腿距在 0.21~0.24 微动 → 正常锁定',
          node.locked, 'locked=%s' % node.locked)


def test_single_leg_fallback():
    print('\n[8] 单腿兜底（只看到一条腿）')

    # ---- 锁定后只剩一条腿 → 不能丢，角度不能跳 ----
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan(legs_objects(0.62)), times=LOCK_N)
    ok = node.locked
    a0 = math.degrees(node.target_angle)
    d0 = node.target_dist

    for _ in range(6):
        feed(node, FakeScan(legs_objects(0.62, hide=0)))
    check('只剩一条腿 → 仍保持跟随', ok and node.target_valid,
          'locked=%s valid=%s lost=%d'
          % (node.locked, node.target_valid, node.tracker.lost_frames))
    check('  角度不被单腿带偏（不左右抽）',
          abs(math.degrees(node.target_angle) - a0) < 2.0,
          '锁定 %.1f° → 现在 %+.1f°' % (a0, math.degrees(node.target_angle)))
    check('  距离仍然跟得上',
          abs(node.target_dist - d0) < 0.05,
          '%.3fm → %.3fm' % (d0, node.target_dist))

    # ---- 单腿距离变化时，距离要跟，角度不跟 ----
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan(legs_objects(0.62)), times=LOCK_N)
    a0 = math.degrees(node.target_angle)
    for i in range(1, 9):
        feed(node, FakeScan(legs_objects(0.62 + 0.04 * i, hide=1)))
    check('单腿距离变大 → 距离跟上',
          abs(node.target_dist - (0.62 + 0.32)) < 0.08,
          '实测 %.3fm，期望 ≈%.2fm' % (node.target_dist, 0.94))
    check('  过程中角度保持不抽',
          abs(math.degrees(node.target_angle) - a0) < 3.0,
          '%.1f° → %+.1f°' % (a0, math.degrees(node.target_angle)))
    check('  目标没有丢', node.target_valid)

    # ---- 单腿兜底不该在未锁定时生效（不能靠一条腿去锁定）----
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan(legs_objects(0.62, hide=0)), times=LOCK_N)
    check('未锁定时单腿不会触发兜底（不会误锁）', not node.locked,
          'locked=%s' % node.locked)

    # ---- 关掉兜底后，只剩一条腿应该丢 ----
    node, rec, _, _ = build_node(leg_fallback_enable=False)
    reset(node)
    feed(node, FakeScan(legs_objects(0.62)), times=LOCK_N)
    ok = node.locked
    for _ in range(6):
        feed(node, FakeScan(legs_objects(0.62, hide=0)))
    check('leg_fallback_enable=false → 只剩一条腿就丢',
          ok and not node.target_valid,
          'locked=%s valid=%s' % (node.locked, node.target_valid))

    # ---- 单腿距离跑太远（超过容差）→ 不认，宁可丢 ----
    node, rec, _, _ = build_node()
    reset(node)
    feed(node, FakeScan(legs_objects(0.62)), times=LOCK_N)
    for _ in range(6):
        feed(node, FakeScan(legs_objects(1.15, hide=0)))
    check('单腿距离突变 0.53m 超过容差 → 判定丢失',
          not node.target_valid,
          'valid=%s dist=%.2f' % (node.target_valid, node.target_dist))


def test_lock_settle():
    print('\n[9] 锁定稳定观察期')

    # 默认关不掉的安全项：这里显式打开
    node, rec, _, _ = build_node(lock_settle_time=1.0)
    reset(node)
    # 先在 0.38m 锁定（必须在 lock_max_range 内），再让它退到 0.85m
    lock_target(node, 0.38)
    settle(node, 0.85)
    check('先锁定 0.38m，再退到 0.85m', node.locked,
          'locked=%s d=%.2f' % (node.locked, node.target_dist))

    # 刚锁定，立刻下指令 → 必须不动
    node.last_ctrl_time -= 0.05
    node.on_ctrl()
    m = rec.last()
    check('刚锁定时输出 0（观察期生效）',
          m is not None and abs(m.linear.x) < 1e-9
          and abs(m.angular.z) < 1e-9,
          'lin=%.3f ang=%.3f' % (m.linear.x, m.angular.z))

    # 把锁定时刻往前推 1.5 秒，模拟观察期结束
    node.lock_time -= 1.5
    node.last_ctrl_time -= 0.05
    node.on_ctrl()
    m = rec.last()
    check('观察期结束后恢复前进',
          m is not None and m.linear.x > 0.0,
          'lin=%.3f 状态=%s' % (m.linear.x, node.state))

    # 关掉观察期 → 锁完立刻能动（验证这个开关本身有效）
    node, rec, _, _ = build_node(lock_settle_time=0.0)
    reset(node)
    lock_target(node, 0.38)
    settle(node, 0.85)
    m = tick(node, rec, n=25)
    check('lock_settle_time=0 时不等待',
          m is not None and m.linear.x > 0.0,
          'lin=%.3f' % (m.linear.x if m else 0))


def main():
    print('=' * 62)
    print('雷达跟随离线仿真测试')
    print('  不创建 ROS 节点、不发布真实消息、不杀任何进程')
    print('=' * 62)

    test_lock_rules()
    test_approach_hold_retreat()
    test_angle_control()
    test_safety()
    test_short_dropout()
    test_smoothing()
    test_leg_pair_lock()
    test_single_leg_fallback()
    test_lock_settle()

    # 清理测试锁文件
    try:
        if os.path.exists('/tmp/test_lidar_follow.lock'):
            os.remove('/tmp/test_lidar_follow.lock')
    except Exception:
        pass

    print('\n' + '=' * 62)
    total = PASS + FAIL
    print('通过 %d/%d' % (PASS, total))
    if FAIL:
        print('❌ 有 %d 项失败' % FAIL)
    else:
        print('✅ 全部通过')
    print('=' * 62)
    return 0 if FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
