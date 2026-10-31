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
LOCK_MAX = 0.80   # 正前方单簇锁定门槛（距离整体 +0.30m）
TRACK_MAX = 1.50
SECTOR_DEG = 30.0
ANG_GAP = math.radians(2.5)
RNG_GAP = 0.15
REAR_DEG = 40.0
REAR_RANGE = 0.40
FRONT_GUARD = 0.32
# 与节点默认值一致的两个防噪参数
MIN_CLUSTER_POINTS = 5
MAX_TARGET_JUMP = 0.30

# ---- 双腿配对（与节点默认值一致）----
PAIR_ENABLE = True
PAIR_LOCK_MAX = 1.30
PAIR_TRACK_MAX = 1.50
PAIR_MAX_SEP = 0.42
PAIR_MIN_SEP = 0.05
PAIR_MAX_DIFF = 0.35
PAIR_MIN_POINTS = 2
PAIR_MAX_SPAN = 45.0
PAIR_MAX_LATERAL = 0.50
PAIR_MAX_ALONG = 0.25
PAIR_REJECT_MULTI = True
PAIR_MULTI_RADIUS = 0.70
# 防误锁
SHAPE_FILTER = True
SINGLE_MAX_SPAN = 35.0
SINGLE_MAX_POINTS = 45
EDGE_MARGIN_DEG = 4.0
SIDE_SECTOR_DEG = 30.0
LOCK_MAX_SIDE = 0.55
LOCK_SECTOR_DEG = 75.0
MAX_ANGLE_JUMP = math.radians(25.0)
CONFIRM_FRAMES = 3
CONFIRM_MISS = 2
CONFIRM_TOL_D = 0.25
CONFIRM_TOL_A = math.radians(12.0)
CONFIRM_TOL_SEP = 0.20
LEG_FALLBACK = True
LEG_MAX_POINTS = 4
LEG_MAX_DIFF = 0.35

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
        # 首次锁定确认计数（与节点一致）
        self.lock_hits = 0
        self.lock_miss = 0
        self.lock_cand = None
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        now = time.monotonic()
        if now - self.last_print < PRINT_EVERY:
            return
        self.last_print = now

        all_pts = GEO.extract_points(msg, TRACK_MAX * 1.15)
        all_clusters = GEO.cluster_points(all_pts, ANG_GAP, RNG_GAP)

        # 未锁定用宽搜索扇区（±75°），已锁定跟着目标方位放宽
        if self.locked:
            half = max(math.radians(SECTOR_DEG),
                       abs(self.target_angle) + math.radians(20.0))
            half = min(half, math.pi)
        else:
            half = math.radians(LOCK_SECTOR_DEG)

        search_pts = GEO.sector_filter(all_pts, half)
        search_clusters = GEO.cluster_points(search_pts, ANG_GAP, RNG_GAP)

        # 窄扇区只给刹车守卫用
        front_pts = GEO.sector_filter(all_pts, math.radians(SECTOR_DEG))
        front_clusters = GEO.cluster_points(front_pts, ANG_GAP, RNG_GAP)

        ref_d = self.target_dist if self.locked else 0.0
        ref_a = self.target_angle if self.locked else 0.0

        pick = GEO.select_follow_target(
            search_clusters, self.locked,
            ref_dist=ref_d, ref_angle=ref_a,
            lock_max_range=LOCK_MAX, track_max_range=TRACK_MAX,
            min_points=MIN_CLUSTER_POINTS,
            max_jump=MAX_TARGET_JUMP,
            max_angle_jump=MAX_ANGLE_JUMP,
            pair_enable=PAIR_ENABLE,
            pair_lock_max_range=PAIR_LOCK_MAX,
            pair_track_max_range=PAIR_TRACK_MAX,
            pair_max_sep=PAIR_MAX_SEP,
            pair_min_sep=PAIR_MIN_SEP,
            pair_max_dist_diff=PAIR_MAX_DIFF,
            pair_min_points=PAIR_MIN_POINTS,
            pair_max_span_deg=PAIR_MAX_SPAN,
            pair_max_lateral_sep=PAIR_MAX_LATERAL,
            pair_max_along_sep=PAIR_MAX_ALONG,
            pair_reject_multi=PAIR_REJECT_MULTI,
            pair_multi_radius=PAIR_MULTI_RADIUS,
            leg_fallback=LEG_FALLBACK,
            leg_max_points=LEG_MAX_POINTS,
            leg_max_dist_diff=LEG_MAX_DIFF,
            shape_filter_enable=SHAPE_FILTER,
            single_max_span_deg=SINGLE_MAX_SPAN,
            single_max_points=SINGLE_MAX_POINTS,
            edge_margin_deg=EDGE_MARGIN_DEG,
            sector_half_rad=half,
            lock_max_range_side=LOCK_MAX_SIDE,
            side_sector_rad=math.radians(SIDE_SECTOR_DEG))

        rear_ok = GEO.rear_is_clear(
            all_clusters, math.radians(REAR_DEG), REAR_RANGE)
        fn = GEO.front_nearest(front_clusters)

        # 全向最近（判断物体是不是在扇区外）
        global_near = None
        if all_clusters:
            global_near = min(all_clusters, key=lambda c: c['dist'])

        ts = time.strftime('%H:%M:%S')
        nfront = len(search_clusters)

        if pick is not None:
            d, a, n = pick['dist'], pick['angle'], pick['n']
            self.target_dist, self.target_angle = d, a

            kind = '双腿对' if pick['kind'] == 'pair' else '单簇  '
            extra = ''
            if pick['kind'] == 'pair':
                la, lb = pick['leg_a'], pick['leg_b']
                extra = ('  [%.2f/%+.0f° + %.2f/%+.0f° 间距%.2f]'
                         % (la[0], math.degrees(la[1]),
                            lb[0], math.degrees(lb[1]), pick['sep']))

            if not self.locked:
                same = False
                if self.lock_cand is not None:
                    cd, ca, cs, ck = self.lock_cand
                    same = (abs(d - cd) <= CONFIRM_TOL_D
                            and abs(GEO.wrap_angle(a - ca)) <= CONFIRM_TOL_A)
                    # 双腿对：间距也要稳定
                    if same and pick['kind'] == 'pair' and ck == 'pair':
                        if abs(pick['sep'] - cs) > CONFIRM_TOL_SEP:
                            same = False
                if same:
                    self.lock_hits += 1
                    self.lock_miss = 0
                    self.lock_cand = (d, a, pick['sep'], pick['kind'])
                else:
                    # 位置/形状变了 → 从这一帧重新计数，不容错。
                    # 容错只留给「整帧看不到目标」。
                    self.lock_cand = (d, a, pick['sep'], pick['kind'])
                    self.lock_hits = 1
                    self.lock_miss = 0

                if self.lock_hits < CONFIRM_FRAMES:
                    print('%s 🔍 候选 %s %.2fm %+.1f° 确认 %d/%d%s'
                          % (ts, kind, d, math.degrees(a),
                             self.lock_hits, CONFIRM_FRAMES, extra))
                    return

                self.locked = True
                self.lock_time = ts
                self.lock_hits = 0
                self.lock_cand = None
                print('\n🔒🔒 %s  【锁定成功】%s %.2fm / %+.1f°（%d点）%s 🔒🔒'
                      % (ts, kind, d, math.degrees(a), n, extra))
            state = ('RETREAT 后退' if d < 0.80 else
                     'HOLD 不动' if d <= 0.95 else
                     'APPROACH 前进')
            print('%s ✅ 锁定 %.2fm %+6.1f° %2d点 %s| %-14s | 后方%s | 前方近%.2fm'
                  % (ts, d, math.degrees(a), n, kind, state,
                     '空' if rear_ok else '有物', fn if fn else -1))
        else:
            if self.locked:
                self.locked = False
                print('%s 💤 【丢失】搜索扇区内无目标' % ts)
            self.lock_miss += 1
            if self.lock_miss > CONFIRM_MISS:
                self.lock_cand = None
                self.lock_hits = 0
                self.lock_miss = 0
            extra = ''
            if global_near is not None:
                gd, ga = global_near['dist'], math.degrees(global_near['angle'])
                if abs(ga) > LOCK_SECTOR_DEG:
                    extra = ('  ← 最近物体在 %.2fm %+.0f°（扇区外，看不到）'
                             % (gd, ga))
                else:
                    extra = ('  ← 最近簇 %.2fm（%d点）超过单簇门槛 %.2fm，'
                             '且未配成双腿对' % (gd, global_near['n'], LOCK_MAX))
            print('%s ⬜ 未锁定  搜索区 %d 点 %d 簇 | 后方%s%s'
                  % (ts, len(search_pts), nfront,
                     '空' if rear_ok else '有物', extra))


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0

    rclpy.init()
    node = LiveMonitor()

    print('=' * 74)
    print('实时雷达监视  —— 只读，不会让小车动')
    print('  单簇锁定门槛 %.2fm(正前) / %.2fm(侧方)  |  '
          '双腿对 %.2fm（间距 <=%.2fm）'
          % (LOCK_MAX, LOCK_MAX_SIDE, PAIR_LOCK_MAX, PAIR_MAX_SEP))
    print('  搜索扇区 ±%.0f°（%s）  跟踪上限 %.2fm  确认 %d 帧'
          % (LOCK_SECTOR_DEG,
             '双腿配对已开' if PAIR_ENABLE else '双腿配对已关',
             TRACK_MAX, CONFIRM_FRAMES))
    print('  请站到车正前方 0.6~1.0m，两条腿分开站好')
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
