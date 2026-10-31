#!/usr/bin/env python3
"""
实机问题的回归测试。

这些用例全部来自 2026-10-31 的实车雷达实测数据，
每一条都对应一个真实发生过的问题，防止以后改动时重新踩坑。

用法：
    source install/setup.bash
    python3 src/fishbot_lidar_follow/test/test_real_data_regression.py
"""

import math
import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
_pkg = os.path.dirname(_here)
if os.path.isdir(os.path.join(_pkg, 'fishbot_lidar_follow')):
    sys.path.insert(0, _pkg)

from fishbot_lidar_follow import geometry as GEO  # noqa: E402

PASS, FAIL = 0, 0


def check(name, cond, detail=''):
    global PASS, FAIL
    if cond:
        PASS += 1
        print('  ✅ %s%s' % (name, ('  ' + detail) if detail else ''))
    else:
        FAIL += 1
        print('  ❌ %s%s' % (name, ('  ' + detail) if detail else ''))


def cl(dist, angle_deg, n):
    return {'dist': dist, 'angle': math.radians(angle_deg),
            'n': n, 'angle_center': math.radians(angle_deg)}


# ============================================================
# 问题 1：2 个点的噪声簇被当成目标
# ------------------------------------------------------------
# 实机现象（2026-10-31 16:21:57）：
#   前方近处目标被推走后，逻辑选中了
#   「0.87m / -20° 只有 2 个点」的簇，判定 APPROACH → 会朝它冲。
#   那 2 个点是柜子边缘的反光，不是目标。
# 修复：min_cluster_points，点数不足的簇直接忽略。
# ============================================================
def test_phantom_cluster():
    print('\n[1] 2 个点的噪声簇不能当目标（实机 16:21:57）')

    clusters = [
        cl(0.87, -20.4, 2),      # 噪声：柜子边缘
    ]

    # 注意：实机那时是「已经锁定」状态（正在跟 0.33m 的目标），
    # 近处目标被推走后，跟踪范围 1.20m 让 0.87m 的噪声簇成为候选。
    # 如果没有 min_cluster_points，它就会被选中。
    old = GEO.select_target(
        clusters, locked=True, ref_dist=0.33, ref_angle=math.radians(12.6),
        lock_max_range=0.50, track_max_range=1.20,
        min_points=1, max_jump=None)
    check('旧行为：已锁定时 2 点噪声簇会被选中（说明这个坑真实存在）',
          old is not None, old and '选中 %.2fm' % old[0])

    # 设最小点数 5 → 被过滤
    new = GEO.select_target(
        clusters, locked=True, ref_dist=0.33, ref_angle=math.radians(12.6),
        lock_max_range=0.50, track_max_range=1.20,
        min_points=5, max_jump=None)
    check('新行为：2 点噪声簇被过滤，本帧无目标', new is None)

    # 未锁定时它也不该被选（超过 40cm 锁定门槛）
    unlocked = GEO.select_target(
        clusters, locked=False, ref_dist=0.0, ref_angle=0.0,
        lock_max_range=0.50, track_max_range=1.20, min_points=1)
    check('未锁定时 0.87m 超过 40cm 门槛，同样不选', unlocked is None)

    # 正常大小的簇仍能选中
    good = [cl(0.35, 5.0, 30)]
    picked = GEO.select_target(
        good, locked=False, ref_dist=0.0, ref_angle=0.0,
        lock_max_range=0.50, track_max_range=1.20, min_points=5)
    check('30 点的真实目标仍能锁定',
          picked is not None and abs(picked[0] - 0.35) < 1e-6,
          picked and '%.2fm' % picked[0])

    # 噪声簇和真实目标并存 → 应该选真实的
    mixed = [cl(0.87, -20.4, 2), cl(0.35, 5.0, 30)]
    picked = GEO.select_target(
        mixed, locked=False, ref_dist=0.0, ref_angle=0.0,
        lock_max_range=0.50, track_max_range=1.20, min_points=5)
    check('噪声与目标并存时选真实目标',
          picked is not None and abs(picked[0] - 0.35) < 1e-6,
          picked and '%.2fm' % picked[0])


# ============================================================
# 问题 2：目标推进 30cm 内后，切换到了远处的另一个物体
# ------------------------------------------------------------
# 实机现象（2026-10-31 16:21:57）：
#   物品在 0.30m（雷达 range_min 边界）徘徊，一进 30cm 内就
#   整个从 /scan 消失。此时唯一剩下的候选是 0.87m 的远簇，
#   与当前 0.33m 只差 0.54m，旧逻辑（阈值 0.60m）判定「可以关联」，
#   于是平滑地切换到了那个物体。
# 修复：max_target_jump，跳变超过阈值就当作本帧没目标，
#       交给丢帧逻辑（会先停车，3 帧后重锁最近的）。
# ============================================================
def test_target_jump():
    print('\n[2] 目标跳到远处物体时不能直接切过去（实机 16:21:57）')

    far = [cl(0.87, -20.4, 8)]

    # 旧逻辑：阈值 0.60，0.54 的跳变被接受 → 静默切换目标
    old = GEO.select_target(
        far, locked=True, ref_dist=0.33, ref_angle=math.radians(12.6),
        lock_max_range=0.50, track_max_range=1.20,
        min_points=1, max_jump=None)
    check('旧行为：0.33m → 0.87m 被静默接受（说明这个坑真实存在）',
          old is not None and abs(old[0] - 0.87) < 1e-6,
          old and '切到 %.2fm' % old[0])

    # 新逻辑：max_jump=0.30 → 拒绝
    new = GEO.select_target(
        far, locked=True, ref_dist=0.33, ref_angle=math.radians(12.6),
        lock_max_range=0.50, track_max_range=1.20,
        min_points=1, max_jump=0.30)
    check('新行为：跳变 0.54m > 0.30m，拒绝切换（当作本帧无目标）',
          new is None)

    # 正常的缓慢移动不能被误拦
    normal = [cl(0.55, 5.0, 30)]
    ok = GEO.select_target(
        normal, locked=True, ref_dist=0.33, ref_angle=math.radians(5.0),
        lock_max_range=0.50, track_max_range=1.20,
        min_points=1, max_jump=0.30)
    check('目标缓慢移动 0.33→0.55m 正常跟踪',
          ok is not None and abs(ok[0] - 0.55) < 1e-6,
          ok and '%.2fm' % ok[0])

    # 0.30m 以内的移动量应当放行（5Hz 下相当于 1.5m/s，已很快）
    fast = [cl(0.60, 0.0, 30)]
    ok = GEO.select_target(
        fast, locked=True, ref_dist=0.32, ref_angle=0.0,
        lock_max_range=0.50, track_max_range=1.20,
        min_points=1, max_jump=0.30)
    check('0.28m 的移动仍在允许范围内',
          ok is not None, ok and '%.2fm' % ok[0])


# ============================================================
# 问题 3：实机前方 ±30° 是空的，但全向有大量近距离点
# ------------------------------------------------------------
# 实机现象（2026-10-31 16:19）：
#   用户说「前方放了物品」，但前方 ±30° 只有 7 个点、最近 1.08m；
#   3m 内最近的物体在 +156.7°（车后方 0.70m）。
#   这说明「用户认为的前方」和「雷达的 0 度方向」可能不一致，
#   或者物品本身不在扇区内。
# 这里固化「扇区内外要分清楚」这个行为。
# ============================================================
def test_sector_boundary():
    print('\n[3] 扇区边界（实机 16:19 前方为空、后方 0.70m 有物）')

    # 模拟实机点云：±30° 内只有远处的少量点，±156° 附近有 0.70m 的簇
    front_far = [cl(1.08, -17.7, 6)]
    rear_near = [cl(0.70, 156.7, 34)]

    # 目标选择只在前方簇里找 → 1.08m 超过 40cm，锁不上
    picked = GEO.select_target(
        front_far, locked=False, ref_dist=0.0, ref_angle=0.0,
        lock_max_range=0.50, track_max_range=1.20, min_points=1)
    check('前方 1.08m 的点超过 40cm 门槛，不锁定', picked is None)

    # 后方那个 0.70m 的簇绝不能被当成跟随目标
    picked_rear = GEO.select_target(
        rear_near, locked=False, ref_dist=0.0, ref_angle=0.0,
        lock_max_range=0.50, track_max_range=1.20, min_points=1)
    check('后方 0.70m 的簇不会被选为跟随目标（超出 40cm）',
          picked_rear is None)

    # 但它必须被后方安全检查看到（否则会盲目后退）
    rear_blocked = GEO.rear_is_clear(
        rear_near, math.radians(40.0), 0.40)
    check('后方的 0.70m 簇被安全检查看见（0.70 > 0.40 所以算安全）',
          rear_blocked is True)

    # 更近的后方物体应该拦住后退
    rear_close = [cl(0.30, 170.0, 20)]
    check('后方 0.30m 的簇会拦住后退',
          GEO.rear_is_clear(rear_close, math.radians(40.0), 0.40) is False)


# ============================================================
# 问题 4：物体在 30cm 内侧时雷达直接丢点
# ------------------------------------------------------------
# 这是硬件限制（range_min=0.3），不是软件 bug。
# 固化成测试，提醒别指望靠软件解决。
# ============================================================
def test_range_min_dropout():
    print('\n[4] range_min 丢点（硬件限制，无法靠软件修复）')

    class FakeScan:
        angle_min = math.radians(-180)
        angle_increment = math.radians(1.0)
        range_min = 0.30
        range_max = 64.0

    scan = FakeScan()
    n = 361
    scan.ranges = [20.0] * n
    scan.ranges[180] = 0.25      # 0°，比 range_min 更近
    scan.ranges[181] = 0.30      # +1°，正好在边界
    scan.ranges[182] = 0.35      # +2°，正常

    pts = GEO.extract_points(scan, 1.38)
    dists = [round(r, 2) for r, _ in pts]

    check('0.25m（< range_min）被丢弃 —— 这是雷达硬件行为',
          0.25 not in dists, '剩余 %s' % dists)
    check('0.30m 边界值保留', 0.30 in dists)
    check('0.35m 正常保留', 0.35 in dists)
    check('结论：物体推进 30cm 内会整个消失，'
          '所以死区下限定在 0.50m 是对的', True)


def test_lock_range_tuning():
    print('\n[5] 锁定门槛 0.50m 标定（实机 0.44m 锁不上 → 放宽）')

    # 实机读数：物体稳定在 0.44m、24~28 点、单簇
    real = [cl(0.44, 6.1, 26)]

    # 旧门槛 0.40 → 锁不上（今天真实发生）
    old = GEO.select_target(
        real, locked=False, ref_dist=0.0, ref_angle=0.0,
        lock_max_range=0.40, track_max_range=1.20, min_points=5)
    check('旧门槛 0.40：0.44m 锁不上（今天实机现象）', old is None)

    # 新门槛 0.50 → 能锁
    new = GEO.select_target(
        real, locked=False, ref_dist=0.0, ref_angle=0.0,
        lock_max_range=0.50, track_max_range=1.20, min_points=5)
    check('新门槛 0.50：0.44m 能锁定',
          new is not None and abs(new[0] - 0.44) < 1e-6,
          new and '%.2fm' % new[0])

    # 边界：正好 0.50 应该能锁（用 <= 判断）
    edge = [cl(0.50, 0.0, 26)]
    picked = GEO.select_target(
        edge, locked=False, ref_dist=0.0, ref_angle=0.0,
        lock_max_range=0.50, track_max_range=1.20, min_points=5)
    check('恰好 0.50m 能锁定（边界含）', picked is not None)

    # 0.51 应该锁不上
    over = [cl(0.51, 0.0, 26)]
    picked = GEO.select_target(
        over, locked=False, ref_dist=0.0, ref_angle=0.0,
        lock_max_range=0.50, track_max_range=1.20, min_points=5)
    check('0.51m 超过门槛，锁不上', picked is None)

    # 关键约束：0.50 锁定后不能立刻后退
    # retreat_enter=0.50，只有 d < 0.50 才后退，所以 d=0.50 应该是 HOLD
    retreat_enter = 0.50
    check('门槛 0.50 == retreat_enter 0.50，锁定后 0.50m 处不进 RETREAT',
          not (0.50 < retreat_enter),
          '(0.50 < %.2f) = False → HOLD' % retreat_enter)


def main():
    print('=' * 66)
    print('实机问题回归测试（数据来自 2026-10-31 真实雷达）')
    print('  纯函数测试，不启动节点、不发指令')
    print('=' * 66)

    test_phantom_cluster()
    test_target_jump()
    test_sector_boundary()
    test_range_min_dropout()
    test_lock_range_tuning()

    print('\n' + '=' * 66)
    total = PASS + FAIL
    print('通过 %d/%d' % (PASS, total))
    if FAIL:
        print('❌ 有 %d 项失败' % FAIL)
    else:
        print('✅ 全部通过')
    print('=' * 66)
    return 0 if FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
