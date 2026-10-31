#!/usr/bin/env python3
"""
实时雷达监视 —— 你移动物品，我实时看能不能锁定。

安全性：
    只订阅 /scan，不发布任何话题，不启动跟随节点。
    /cmd_vel 完全不碰，小车不会动。

它调用 fishbot_lidar_follow 包里的真实几何函数，
所以"能否锁定"的判断和正式运行完全一致。

用法：
    python3 probe_scan_live.py            # 监视 30 秒
    python3 probe_scan_live.py 60         # 监视 60 秒
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from fishbot_lidar_follow import geometry as GEO

# 与 config/lidar_follow.yaml 默认值一致
LOCK_MAX = 0.50   # 实机标定：0.44m 时点云最稳，0.40 差 4cm
TRACK_MAX = 1.20
SECTOR_DEG = 30.0
ANG_GAP = math.radians(2.5)
RNG_GAP = 0.15
REAR_DEG = 40.0
REAR_RANGE = 0.40
FRONT_GUARD = 0.32
# 与节点默认值一致的两个防噪参数
MIN_CLUSTER_POINTS = 5
MAX_TARGET_JUMP = 0.30

# 打印间隔（秒）。雷达 5Hz，每帧都印会刷屏
PRINT_EVERY = 0.4


class LiveMonitor(Node):
    def __init__(self):
        super().__init__('scan_live_readonly')
        self.last_print = 0.0
        self.locked = False
        self.lock_time = None
        self.target_dist = 0.0
        self.target_angle = 0.0
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        now = time.monotonic()
        if now - self.last_print < PRINT_EVERY:
            return
        self.last_print = now

        sector_rad = math.radians(SECTOR_DEG)

        all_pts = GEO.extract_points(msg, TRACK_MAX * 1.15)
        all_clusters = GEO.cluster_points(all_pts, ANG_GAP, RNG_GAP)
        front_pts = GEO.sector_filter(all_pts, sector_rad)
        front_clusters = GEO.cluster_points(front_pts, ANG_GAP, RNG_GAP)

        # 用与节点完全一致的逻辑（含防噪与跳变保护）
        ref_d = self.target_dist if self.locked else 0.0
        ref_a = self.target_angle if self.locked else 0.0
        pick = GEO.select_target(
            front_clusters, self.locked,
            ref_dist=ref_d, ref_angle=ref_a,
            lock_max_range=LOCK_MAX, track_max_range=TRACK_MAX,
            min_points=MIN_CLUSTER_POINTS,
            max_jump=MAX_TARGET_JUMP)

        rear_ok = GEO.rear_is_clear(
            all_clusters, math.radians(REAR_DEG), REAR_RANGE)
        fn = GEO.front_nearest(front_clusters)

        # 全向最近（判断物体是不是在扇区外）
        global_near = None
        if all_clusters:
            global_near = min(all_clusters, key=lambda c: c['dist'])

        ts = time.strftime('%H:%M:%S')
        nfront = len(front_clusters)

        if pick is not None:
            d, a, n = pick
            self.target_dist, self.target_angle = d, a
            if not self.locked:
                self.locked = True
                self.lock_time = ts
                print('\n🔒🔒 %s  【锁定成功】%.2fm / %+.1f°（%d点）🔒🔒'
                      % (ts, d, math.degrees(a), n))
            state = ('RETREAT 后退' if d < 0.50 else
                     'HOLD 不动' if d <= 0.65 else
                     'APPROACH 前进')
            print('%s ✅ 锁定 %.2fm %+6.1f° %2d点 | %-14s | 后方%s | 前方近%.2fm'
                  % (ts, d, math.degrees(a), n, state,
                     '空' if rear_ok else '有物', fn if fn else -1))
        else:
            if self.locked:
                self.locked = False
                print('%s 💤 【丢失】前方扇区内无目标' % ts)
            extra = ''
            if global_near is not None:
                gd, ga = global_near['dist'], math.degrees(global_near['angle'])
                if abs(ga) > SECTOR_DEG:
                    extra = ('  ← 最近物体在 %.2fm %+.0f°（扇区外，看不到）'
                             % (gd, ga))
                else:
                    extra = ('  ← 最近 %.2fm 超过 %.2fm 锁定门槛'
                             % (gd, LOCK_MAX))
            elif nfront:
                extra = '  ← 有 %d 簇但都超过 %.2fm' % (nfront, LOCK_MAX)
            print('%s ⬜ 未锁定  前方 %d 点 %d 簇 | 后方%s%s'
                  % (ts, len(front_pts), nfront,
                     '空' if rear_ok else '有物', extra))


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0

    rclpy.init()
    node = LiveMonitor()

    print('=' * 74)
    print('实时雷达监视  —— 只读，不会让小车动')
    print('  锁定门槛 %.2fm  扇区 ±%.0f°  跟踪上限 %.2fm'
          % (LOCK_MAX, SECTOR_DEG, TRACK_MAX))
    print('  请把物品放到 车正前方 40cm 以内')
    print('  监视 %.0f 秒，Ctrl+C 可提前结束' % duration)
    print('=' * 74)

    t0 = time.time()
    try:
        while time.time() - t0 < duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass

    print('=' * 74)
    print('监视结束。本脚本未发布任何指令。')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
