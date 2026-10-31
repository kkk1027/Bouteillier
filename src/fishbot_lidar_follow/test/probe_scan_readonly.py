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
LOCK_MAX_RANGE = 0.50   # 实机标定，见 config 注释
TRACK_MAX_RANGE = 1.20
SECTOR_DEG = 30.0
CLUSTER_ANGLE_GAP = math.radians(2.5)
CLUSTER_RANGE_GAP = 0.15
REAR_DEG = 40.0
REAR_RANGE = 0.40
FRONT_GUARD = 0.32
# 与节点默认值一致的两个防噪参数（实机验证后新增）
MIN_CLUSTER_POINTS = 5
MAX_TARGET_JUMP = 0.30


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

    # 前方扇区（目标选择用）
    front_pts = GEO.sector_filter(all_pts, sector_rad)
    front_clusters = GEO.cluster_points(
        front_pts, CLUSTER_ANGLE_GAP, CLUSTER_RANGE_GAP)

    # 用与节点完全一致的逻辑（含防噪与跳变保护）
    pick = GEO.select_target(
        front_clusters, locked=False, ref_dist=0.0, ref_angle=0.0,
        lock_max_range=LOCK_MAX_RANGE, track_max_range=TRACK_MAX_RANGE,
        min_points=MIN_CLUSTER_POINTS,
        max_jump=None)   # 未锁定时不需要跳变保护

    rear_ok = GEO.rear_is_clear(
        all_clusters, math.radians(REAR_DEG), REAR_RANGE)
    front_near = GEO.front_nearest(front_clusters)

    return {
        'all_pts': len(all_pts),
        'front_pts': len(front_pts),
        'front_clusters': front_clusters,
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
            inlock = '★可锁定' if c['dist'] <= LOCK_MAX_RANGE else '  超出门槛'
            print('   %.2fm  %+6.1f°  %2d点  %s'
                  % (c['dist'], math.degrees(c['angle']), c['n'], inlock))

    print('\n' + '-' * 66)
    if r['pick'] is not None:
        d, a, n = r['pick']
        print('✅ 能锁定目标：%.2fm / %+.1f°（%d 点）' % (d, math.degrees(a), n))
        if d <= 0.50:
            print('   → 距离 %.2f < 0.50，会进入 RETREAT（后退）' % d)
        elif d <= 0.65:
            print('   → 距离 %.2f 在死区 0.55~0.65，会 HOLD（不动）' % d)
        else:
            print('   → 距离 %.2f > 0.65，会 APPROACH（前进）' % d)
    else:
        print('❌ 无法锁定：前方 ±%.0f° 内 %.2fm 以内没有物体' %
              (SECTOR_DEG, LOCK_MAX_RANGE))
        if r['front_nearest'] is not None:
            print('   最近的物体在 %.2fm（超过 %.2fm 锁定门槛）'
                  % (r['front_nearest'], LOCK_MAX_RANGE))
            print('   → 把物体往前挪到 %.2fm 以内就能锁定' % LOCK_MAX_RANGE)

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
