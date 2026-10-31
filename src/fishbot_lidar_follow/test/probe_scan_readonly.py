#!/usr/bin/env python3
"""
只读的雷达数据分析 —— 判断「能否锁定目标」。

安全性：
    只订阅 /scan，不发布任何话题，不启动跟随节点，
    参数里没有任何 cmd_vel。全程不会让小车动一下。

它直接调用 fishbot_lidar_follow 包里的真实几何函数
（extract_points / sector_filter / cluster_points / select_target），
所以结论就是真正运行时的结论。
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from fishbot_lidar_follow import geometry as GEO

SCAN_TOPIC = '/scan'
N_SAMPLES = 5           # 采几帧
TIMEOUT = 8.0

# 与 config/lidar_follow.yaml 的默认值保持一致
LOCK_MAX_RANGE = 0.80   # 正前方单簇锁定门槛（距离整体 +0.30m）
TRACK_MAX_RANGE = 1.50
LOCK_MAX_SIDE = 0.55     # 侧方门槛更紧，防墙面误锁
SIDE_SECTOR_DEG = 30.0
SECTOR_DEG = 30.0
LOCK_SECTOR_DEG = 75.0   # 未锁定时的搜索扇区半角（和节点一致）
CLUSTER_ANGLE_GAP = math.radians(2.5)
CLUSTER_RANGE_GAP = 0.15
REAR_DEG = 40.0
REAR_RANGE = 0.40
FRONT_GUARD = 0.62       # 随跟随距离整体 +0.30
# 与节点默认值一致的防噪参数（实机验证后新增）
MIN_CLUSTER_POINTS = 5
MAX_TARGET_JUMP = 0.30

# 防误锁（墙 / 椅脚）
SHAPE_FILTER = True
SINGLE_MAX_SPAN = 35.0
SINGLE_MAX_POINTS = 45
EDGE_MARGIN = 4.0

# 双腿配对
PAIR_LOCK_MAX = 1.30
PAIR_TRACK_MAX = 1.50
PAIR_MAX_SEP = 0.42
PAIR_MIN_SEP = 0.05
PAIR_MAX_DIFF = 0.22
PAIR_MIN_POINTS = 2
PAIR_MAX_SPAN = 45.0
PAIR_MAX_LATERAL = 0.50
PAIR_MAX_ALONG = 0.25


class ScanProbe(Node):
    def __init__(self):
        super().__init__('scan_probe_readonly')
        self.frames = []
        self.create_subscription(
            LaserScan, SCAN_TOPIC, self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        self.frames.append(msg)


def describe(msg):
    n = len(msg.ranges)
    valid = sum(1 for r in msg.ranges
                if not (math.isnan(r) or math.isinf(r)))
    return ('angle_min=%.1f° angle_max=%.1f° inc=%.2f° '
            'n=%d range=[%.2f,%.1f] frame=%s'
            % (math.degrees(msg.angle_min), math.degrees(msg.angle_max),
               math.degrees(msg.angle_increment), n,
               msg.range_min, msg.range_max, msg.header.frame_id)), valid


def analyse(msg):
    sector_rad = math.radians(SECTOR_DEG)

    # 全向点（安全检查用）
    all_pts = GEO.extract_points(msg, TRACK_MAX_RANGE * 1.15)
    all_clusters = GEO.cluster_points(
        all_pts, CLUSTER_ANGLE_GAP, CLUSTER_RANGE_GAP)

    # 搜索扇区（目标选择用）。未锁定时用宽扇区，和节点一致。
    search_rad = math.radians(LOCK_SECTOR_DEG)
    search_pts = GEO.sector_filter(all_pts, search_rad)
    search_clusters = GEO.cluster_points(
        search_pts, CLUSTER_ANGLE_GAP, CLUSTER_RANGE_GAP)

    # 窄前方扇区（刹车守卫用）
    front_pts = GEO.sector_filter(all_pts, sector_rad)
    front_clusters = GEO.cluster_points(
        front_pts, CLUSTER_ANGLE_GAP, CLUSTER_RANGE_GAP)

    # 用与节点完全一致的逻辑（含防误锁、防噪、双腿配对）
    pick = GEO.select_follow_target(
        search_clusters, False, 0.0, 0.0,
        LOCK_MAX_RANGE, TRACK_MAX_RANGE,
        min_points=MIN_CLUSTER_POINTS,
        max_jump=None,
        pair_enable=True,
        pair_lock_max_range=PAIR_LOCK_MAX,
        pair_track_max_range=PAIR_TRACK_MAX,
        pair_max_sep=PAIR_MAX_SEP,
        pair_min_sep=PAIR_MIN_SEP,
        pair_max_dist_diff=PAIR_MAX_DIFF,
        pair_min_points=PAIR_MIN_POINTS,
        pair_max_span_deg=PAIR_MAX_SPAN,
        pair_max_lateral_sep=PAIR_MAX_LATERAL,
        pair_max_along_sep=PAIR_MAX_ALONG,
        pair_reject_multi=True,
        pair_multi_radius=0.70,
        leg_fallback=False,
        shape_filter_enable=SHAPE_FILTER,
        single_max_span_deg=SINGLE_MAX_SPAN,
        single_max_points=SINGLE_MAX_POINTS,
        edge_margin_deg=EDGE_MARGIN,
        sector_half_rad=search_rad,
        lock_max_range_side=LOCK_MAX_SIDE,
        side_sector_rad=math.radians(SIDE_SECTOR_DEG))

    rear_ok = GEO.rear_is_clear(
        all_clusters, math.radians(REAR_DEG), REAR_RANGE)
    front_near = GEO.front_nearest(front_clusters)

    return {
        'all_pts': len(all_pts),
        'front_pts': len(search_pts),
        'front_clusters': search_clusters,
        'all_clusters': all_clusters,
        'pick': pick,
        'rear_clear': rear_ok,
        'front_nearest': front_near,
    }


def main():
    rclpy.init()
    node = ScanProbe()

    t0 = time.time()
    while len(node.frames) < N_SAMPLES and time.time() - t0 < TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.3)

    if not node.frames:
        print('❌ %.0f 秒内没收到任何 /scan 数据' % TIMEOUT)
        print('   检查雷达：ros2 topic hz /scan')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    msg = node.frames[-1]
    print('=' * 66)
    print('收到 %d 帧 /scan' % len(node.frames))
    head, valid = describe(msg)
    print('  %s' % head)
    print('  有效点 %d / %d' % (valid, len(msg.ranges)))

    r = analyse(msg)
    print('=' * 66)
    print('提取（全向，距离<=%.2fm）: %d 点' % (TRACK_MAX_RANGE * 1.15,
                                          r['all_pts']))
    print('前方 ±%.0f° 扇区内       : %d 点 → %d 个簇'
          % (SECTOR_DEG, r['front_pts'], len(r['front_clusters'])))

    if r['front_clusters']:
        print('\n前方物体（按距离排序）:')
        for c in sorted(r['front_clusters'], key=lambda x: x['dist']):
            lim = (LOCK_MAX_SIDE if abs(c['angle']) > math.radians(
                SIDE_SECTOR_DEG) else LOCK_MAX_RANGE)
            inlock = '★可锁定' if c['dist'] <= lim else '  超出门槛'
            print('   %.2fm  %+6.1f°  %2d点  %s'
                  % (c['dist'], math.degrees(c['angle']), c['n'], inlock))

    print('\n' + '-' * 66)
    if r['pick'] is not None:
        p = r['pick']
        d, a, n = p['dist'], p['angle'], p['n']
        kind = {'pair': '双腿对', 'single': '单簇', 'leg': '单腿'}.get(
            p['kind'], p['kind'])
        extra = ''
        if p['kind'] == 'pair':
            la, lb = p['leg_a'], p['leg_b']
            extra = ('  [%.2f/%+.0f° + %.2f/%+.0f° 间距%.2f]'
                     % (la[0], math.degrees(la[1]),
                        lb[0], math.degrees(lb[1]), p['sep']))
        print('✅ 能锁定目标【%s】：%.2fm / %+.1f°（%d 点）%s'
              % (kind, d, math.degrees(a), n, extra))
        if d < 0.80:
            print('   → 距离 %.2f < 0.80，会进入 RETREAT（后退）' % d)
        elif d <= 0.95:
            print('   → 距离 %.2f 在死区 0.80~0.95，会 HOLD（不动）' % d)
        else:
            print('   → 距离 %.2f > 0.95，会 APPROACH（前进）' % d)
    else:
        print('❌ 无法锁定：搜索扇区 ±%.0f° 内没有合格目标'
              % LOCK_SECTOR_DEG)
        print('   门槛：正前方 ±%.0f° 内 %.2fm / 侧方 %.2fm'
              % (SIDE_SECTOR_DEG, LOCK_MAX_RANGE, LOCK_MAX_SIDE))
        print('   另外还会拦截：过宽/过多点/顶到扇区边界的簇（防墙）、')
        print('                 多腿共现（防椅子）、间距 >%.2fm 的假双腿'
              % PAIR_MAX_SEP)
        if r['front_nearest'] is not None:
            print('   前方最近的物体在 %.2fm' % r['front_nearest'])

    print('\n安全检查状态:')
    print('   后方 %.1fm 内 %s' % (REAR_RANGE,
                                '无遮挡 ✅' if r['rear_clear'] else '有障碍 ⚠️'))
    fn = r['front_nearest']
    print('   前方最近 %.2fm %s' % (fn if fn is not None else -1,
                                '（低于 %.2fm 会刹车 ⚠️）' % FRONT_GUARD
                                if fn is not None and fn < FRONT_GUARD
                                else '（安全）'))
    print('=' * 66)
    print('本脚本只读，未发布任何指令。')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
