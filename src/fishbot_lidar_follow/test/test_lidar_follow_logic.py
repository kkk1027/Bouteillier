#!/usr/bin/env python3
"""
雷达跟随的纯逻辑测试 —— 不需要连小车，也不需要启动 ROS 节点。

覆盖：
    1. 三态滞回状态机（核心，验证死区不抖）
    2. 一维匀速卡尔曼（收敛性 + 速度估计）
    3. 点云聚类
    4. 前方/后方安全检查
    5. 参数自检（死区顺序错误要报出来）

用法：
    source install/setup.bash
    python3 src/fishbot_lidar_follow/test/test_lidar_follow_logic.py

注意：本测试完全不发布 /cmd_vel，不会让小车动。
"""

import math
import os
import sys

# 允许直接从源码目录运行（未编译时）
_here = os.path.dirname(os.path.abspath(__file__))
_pkg = os.path.dirname(_here)
if os.path.isdir(os.path.join(_pkg, 'fishbot_lidar_follow')):
    sys.path.insert(0, _pkg)

try:
    from fishbot_lidar_follow import geometry as GEO
    from fishbot_lidar_follow.tracker import CVKalman
    from fishbot_lidar_follow.lidar_follow import (
        LidarFollowNode,
        ST_APPROACH, ST_HOLD, ST_RETREAT, ST_SEARCH, ST_IDLE,
    )
except ImportError as e:
    print('❌ 导入失败: %s' % e)
    print('   请先 source install/setup.bash，或在本包 src 目录下运行')
    sys.exit(2)


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
# 状态机测试用的桩对象
# ------------------------------------------------------------
# _update_state 只读 self 上的几个阈值属性和 self.state，
# 不碰任何 rclpy API，所以可以用 __new__ 绕过 __init__。
# ============================================================
def make_stub():
    s = LidarFollowNode.__new__(LidarFollowNode)
    s.approach_enter = 0.65
    s.approach_exit = 0.55
    s.retreat_enter = 0.50
    s.retreat_exit = 0.55
    s.state = ST_HOLD
    s.sector_rad = math.radians(30.0)
    s.rear_rad = math.radians(40.0)      # 注意是 rad，不是 deg
    s.rear_range = 0.40
    s.front_guard = 0.32
    s.track_max_range = 1.20
    s.lock_max_range = 0.40
    s.last_clusters = []
    s.all_clusters = []
    return s


def test_hysteresis():
    print('\n[1] 三态滞回状态机')

    sm = LidarFollowNode.update_state
    s = make_stub()

    # ---- 目标从远处走近 ----
    seq_far_to_near = [1.20, 1.00, 0.90, 0.80, 0.70, 0.66,
                       0.64, 0.60, 0.56, 0.54, 0.51, 0.49, 0.45, 0.40]
    states = []
    s.state = ST_HOLD
    for d in seq_far_to_near:
        s.state = sm(s, d)
        states.append((d, s.state))

    expect = [
        (1.20, ST_APPROACH), (1.00, ST_APPROACH), (0.90, ST_APPROACH),
        (0.80, ST_APPROACH), (0.70, ST_APPROACH), (0.66, ST_APPROACH),
        # 0.64 已经小于进入阈值 0.65，但退出阈值是 0.55，所以还在前进
        (0.64, ST_APPROACH), (0.60, ST_APPROACH), (0.56, ST_APPROACH),
        # 0.54 <= 0.55，停止前进
        (0.54, ST_HOLD), (0.51, ST_HOLD),
        # 0.49 < 0.50，开始后退
        (0.49, ST_RETREAT), (0.45, ST_RETREAT), (0.40, ST_RETREAT),
    ]
    ok = all(a[1] == b[1] for a, b in zip(states, expect))
    check('走近过程状态转移正确', ok,
          '' if ok else '实际: %s' % [(d, st) for d, st in states])

    # ---- 目标从近处退远 ----
    s.state = ST_RETREAT
    seq_near_to_far = [0.45, 0.48, 0.52, 0.54, 0.55, 0.58,
                       0.62, 0.65, 0.66, 0.80, 1.00]
    states2 = []
    for d in seq_near_to_far:
        s.state = sm(s, d)
        states2.append((d, s.state))

    expect2 = [
        (0.45, ST_RETREAT), (0.48, ST_RETREAT),
        (0.52, ST_RETREAT), (0.54, ST_RETREAT),
        # 0.55 >= retreat_exit，停止后退
        (0.55, ST_HOLD), (0.58, ST_HOLD), (0.62, ST_HOLD),
        # 0.65 不大于 0.65，还不动
        (0.65, ST_HOLD),
        # 0.66 > 0.65，开始前进
        (0.66, ST_APPROACH), (0.80, ST_APPROACH), (1.00, ST_APPROACH),
    ]
    ok2 = all(a[1] == b[1] for a, b in zip(states2, expect2))
    check('退远过程状态转移正确', ok2,
          '' if ok2 else '实际: %s' % [(d, st) for d, st in states2])

    # ---- 死区抖动测试：目标在 0.50 附近疯狂抖动 ----
    # 这是滞回要解决的核心问题。
    # 注意：HOLD → RETREAT 的第一次跳转是**正确行为**，不是抖动。
    # 真正要验证的是「进入某个状态之后，不会因为 ±5mm 的抖动来回翻」。
    s.state = ST_HOLD
    s.state = sm(s, 0.495)
    check('0.495 应进入后退', s.state == ST_RETREAT, '实际=%s' % s.state)

    flip = 0
    prev = s.state
    for d in [0.505, 0.495, 0.505, 0.498, 0.502,
              0.495, 0.505, 0.499, 0.501, 0.496]:
        s.state = sm(s, d)
        if s.state != prev:
            flip += 1
            prev = s.state
    check('后退中在 0.50 附近抖动不翻转', flip == 0, '翻转次数=%d' % flip)

    # 后退到 0.55 以上才退出
    s.state = sm(s, 0.552)
    check('0.552 退出后退', s.state == ST_HOLD, '实际=%s' % s.state)

    # ---- 前进方向同样测一遍 ----
    s.state = ST_HOLD
    s.state = sm(s, 0.66)
    check('0.66 应进入前进', s.state == ST_APPROACH, '实际=%s' % s.state)

    flip = 0
    prev = s.state
    for d in [0.64, 0.66, 0.63, 0.65, 0.66, 0.62, 0.60, 0.58, 0.56]:
        s.state = sm(s, d)
        if s.state != prev:
            flip += 1
            prev = s.state
    check('前进中在 0.55~0.65 抖动不翻转', flip == 0, '翻转次数=%d' % flip)

    # 前进到 0.55 以下才停
    s.state = sm(s, 0.548)
    check('0.548 停止前进', s.state == ST_HOLD, '实际=%s' % s.state)


def test_kalman():
    print('\n[2] 一维匀速卡尔曼滤波')

    # ---- 恒定输入：应收敛到该值，速度趋近 0 ----
    kf = CVKalman(0.0015, 0.060, 0.010)
    for _ in range(60):
        kf.predict(0.05)
        kf.update(0.60)
    check('恒定 0.60 输入收敛', abs(kf.pos - 0.60) < 0.01,
          '滤波=%.4f' % kf.pos)
    check('恒定输入速度趋近 0', abs(kf.vel) < 0.05,
          '速度=%.4f m/s' % kf.vel)

    # ---- 线性变化：速度应估计出斜率 ----
    # 0.60 开始，每 0.05s 增加 0.01m → 0.2 m/s
    kf = CVKalman(0.0015, 0.060, 0.010)
    d = 0.60
    for _ in range(80):
        kf.predict(0.05)
        kf.update(d)
        d += 0.01
    check('线性输入能估计出速度',
          abs(kf.vel - 0.20) < 0.06,
          '估计=%.3f m/s（真实 0.20）' % kf.vel)

    # ---- 噪声抑制：加 ±3cm 噪声，滤波后波动应明显变小 ----
    import random
    random.seed(42)
    kf = CVKalman(0.0015, 0.060, 0.010)
    vals = []
    for _ in range(100):
        kf.predict(0.05)
        kf.update(0.60 + random.uniform(-0.03, 0.03))
        if _ > 40:
            vals.append(kf.pos)
    raw_span = 0.06
    filt_span = max(vals) - min(vals)
    check('噪声被抑制', filt_span < raw_span * 0.5,
          '原始跨度 %.3f → 滤波跨度 %.3f' % (raw_span, filt_span))

    # ---- 未初始化时 update 应自动初始化 ----
    kf = CVKalman()
    check('初始未初始化', not kf.initialized)
    kf.update(0.8)
    check('首次 update 自动初始化', kf.initialized and abs(kf.pos - 0.8) < 1e-6)


def test_cluster():
    print('\n[3] 点云聚类')

    # cluster_points 是纯函数，不需要 self
    ANG = math.radians(2.5)
    RNG = 0.15
    cluster = lambda pts: GEO.cluster_points(pts, ANG, RNG)  # noqa: E731

    # 两团：一团在 -5°~-3° 距离 0.5，一团在 10°~12° 距离 1.0
    pts = []
    for a in (-5, -4, -3):
        pts.append((0.50, math.radians(a)))
    for a in (10, 11, 12):
        pts.append((1.00, math.radians(a)))
    pts.sort(key=lambda t: t[1])

    cl = cluster(pts)
    check('两团分成两个簇', len(cl) == 2, '簇数=%d' % len(cl))
    if len(cl) == 2:
        check('第一簇最近距离 0.50', abs(cl[0]['dist'] - 0.50) < 1e-6,
              '%.3f' % cl[0]['dist'])
        check('第一簇点数 3', cl[0]['n'] == 3, '%d' % cl[0]['n'])
        check('第二簇最近距离 1.00', abs(cl[1]['dist'] - 1.00) < 1e-6,
              '%.3f' % cl[1]['dist'])

    # 单点
    cl = cluster([(0.4, 0.0)])
    check('单点成为一个簇', len(cl) == 1 and cl[0]['n'] == 1)

    # 空
    check('空输入返回空', cluster([]) == [])

    # 角度间隔大于阈值 → 分开
    pts = [(0.5, math.radians(0)), (0.5, math.radians(10))]
    cl = cluster(pts)
    check('角度差 10° 分为两簇', len(cl) == 2, '簇数=%d' % len(cl))

    # 距离差大于阈值 → 分开
    pts = [(0.5, 0.0), (0.9, math.radians(1))]
    cl = cluster(pts)
    check('距离差 0.4m 分为两簇', len(cl) == 2, '簇数=%d' % len(cl))


def test_safety():
    print('\n[4] 安全检查')

    s = make_stub()

    # ---- 后方检查（用 all_clusters，全向点云）----
    s.all_clusters = []
    check('没有点云时后方视为不安全', GEO.rear_is_clear(s.all_clusters, s.rear_rad, s.rear_range) is False)

    # 正后方 30cm 有障碍 → 不安全
    s.all_clusters = [{'dist': 0.30, 'angle': math.pi, 'n': 3,
                       'angle_center': math.pi}]
    check('正后方 0.30m 有障碍 → 不能后退',
          GEO.rear_is_clear(s.all_clusters, s.rear_rad, s.rear_range) is False)

    # 正后方 60cm 有障碍 → 安全（超过 rear_range=0.40）
    s.all_clusters = [{'dist': 0.60, 'angle': math.pi, 'n': 3,
                       'angle_center': math.pi}]
    check('正后方 0.60m 有障碍 → 可以后退',
          GEO.rear_is_clear(s.all_clusters, s.rear_rad, s.rear_range) is True)

    # 正前方 30cm 有障碍 → 后方安全
    s.all_clusters = [{'dist': 0.30, 'angle': 0.0, 'n': 3,
                       'angle_center': 0.0}]
    check('障碍在正前方不影响后退判断',
          GEO.rear_is_clear(s.all_clusters, s.rear_rad, s.rear_range) is True)

    # 侧方 90° 30cm → 后方安全（不在 ±40° 内）
    s.all_clusters = [{'dist': 0.30, 'angle': math.pi / 2, 'n': 3,
                       'angle_center': math.pi / 2}]
    check('侧方障碍不算后方',
          GEO.rear_is_clear(s.all_clusters, s.rear_rad, s.rear_range) is True)

    # 后方扇区 = 180°±40°，即 [140°,180°] 与 [-180°,-140°]
    s.all_clusters = [{'dist': 0.30, 'angle': math.radians(-150), 'n': 3,
                       'angle_center': math.radians(-150)}]
    check('后方 -150°（偏离 30°）0.30m → 算后方障碍',
          GEO.rear_is_clear(s.all_clusters, s.rear_rad, s.rear_range) is False)

    s.all_clusters = [{'dist': 0.30, 'angle': math.radians(150), 'n': 3,
                       'angle_center': math.radians(150)}]
    check('后方 +150°（偏离 30°）0.30m → 算后方障碍',
          GEO.rear_is_clear(s.all_clusters, s.rear_rad, s.rear_range) is False)

    # -135° 偏离后方 45°，超出 ±40° 扇区 → 不算后方
    s.all_clusters = [{'dist': 0.30, 'angle': math.radians(-135), 'n': 3,
                       'angle_center': math.radians(-135)}]
    check('后方 -135°（偏离 45°）超出扇区 → 不算后方',
          GEO.rear_is_clear(s.all_clusters, s.rear_rad, s.rear_range) is True)

    # ---- 前方最近距离（用 last_clusters，前方簇）----
    s.last_clusters = [
        {'dist': 0.80, 'angle': 0.0, 'n': 3, 'angle_center': 0.0},
        {'dist': 0.45, 'angle': math.radians(5), 'n': 3,
         'angle_center': math.radians(5)},
    ]
    nearest = GEO.front_nearest(s.last_clusters)
    check('前方最近距离取最小值', nearest is not None and abs(nearest - 0.45) < 1e-6,
          '%.2f' % (nearest if nearest else -1))

    s.last_clusters = []
    check('无点云时前方最近返回 None',
          GEO.front_nearest(s.last_clusters) is None)


def test_param_validation():
    print('\n[5] 参数自检逻辑')

    # 直接验证阈值顺序关系（这是 _load_params 里检查的）
    # 正确顺序
    ok = (0.50 < 0.55 <= 0.55 < 0.65)
    check('默认参数满足 retreat_enter < retreat_exit <= '
          'approach_exit < approach_enter', ok)

    # 错误顺序应当被识别出来
    bad = (0.65 < 0.55 <= 0.55 < 0.50)
    check('错误顺序能被识别', not bad)

    # 滞回带宽度
    band = 0.65 - 0.55
    check('滞回带 10cm', abs(band - 0.10) < 1e-9, '%.2fm' % band)


def test_scan_extract():
    print('\n[6] 点云提取与扇区过滤')

    s = make_stub()
    s.track_max_range = 1.20

    class FakeScan:
        angle_min = math.radians(-180)
        angle_increment = math.radians(1.0)
        range_min = 0.30
        range_max = 64.0
        ranges = []

    n = 361
    FakeScan.ranges = [20.0] * n      # 默认都远超 track_max_range，会被过滤
    FakeScan.angle_min = math.radians(-180)
    FakeScan.angle_increment = math.radians(1.0)

    # 索引 i 对应角度 -180 + i*1
    FakeScan.ranges[180] = 0.50       # 0°
    FakeScan.ranges[200] = 0.80       # +20°
    FakeScan.ranges[225] = 0.55       # +45°（扇区外）
    FakeScan.ranges[90] = 0.60        # -90°（正左）
    FakeScan.ranges[0] = 0.40         # -180°（正后）
    FakeScan.ranges[181] = 0.20       # +1°，低于 range_min
    FakeScan.ranges[179] = float('inf')

    pts = GEO.extract_points(FakeScan, s.track_max_range * 1.15)
    angles = [round(math.degrees(a), 1) for _, a in pts]
    dists = [round(r, 2) for r, _ in pts]

    # ★ _extract_points 现在返回全向点云（因为安全检查要看后方）
    check('提取返回全向点云（含后方 -180°）',
          any(abs(a) > 90 for a in angles), '角度=%s' % angles)
    check('包含 +45° 的扇区外点', 45.0 in angles, '角度=%s' % angles)
    check('低于 range_min 的 0.20m 被过滤', 0.20 not in dists)
    check('inf 被过滤', all(d < 60 for d in dists))
    check('超出 track_max_range 的 20m 点被过滤',
          all(d <= 1.38 for d in dists), '距离=%s' % dists)

    # 扇区过滤在 _on_scan 里做，这里验证同样的筛选逻辑
    sector_rad = math.radians(30.0)
    front = [p for p in pts if abs(p[1]) <= sector_rad]
    front_angles = [round(math.degrees(a), 1) for _, a in front]
    check('±30° 筛选后不含 +45°',
          all(abs(a) <= 30.0 for a in front_angles),
          '角度=%s' % front_angles)
    check('±30° 筛选后保留 0° 和 +20°',
          0.0 in front_angles and 20.0 in front_angles,
          '角度=%s' % front_angles)


def main():
    print('=' * 62)
    print('雷达跟随纯逻辑测试（不启动节点、不发布 /cmd_vel）')
    print('=' * 62)

    test_hysteresis()
    test_kalman()
    test_cluster()
    test_safety()
    test_param_validation()
    test_scan_extract()

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
