#!/usr/bin/env python3
"""
只读的雷达全景诊断 —— 打印 360° 点云分布，找出物体的实际方位。

安全性：只订阅 /scan，不发布任何话题，不会让小车动。
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

SCAN_TOPIC = '/scan'
N_SAMPLES = 5
TIMEOUT = 8.0
MAX_RANGE = 3.0          # 诊断范围放宽到 3m

SECTORS = [(-180, -150), (-150, -120), (-120, -90), (-90, -60),
           (-60, -30), (-30, 0), (0, 30), (30, 60), (60, 90),
           (90, 120), (120, 150), (150, 180)]


class Probe(Node):
    def __init__(self):
        super().__init__('scan_panorama_readonly')
        self.frames = []
        self.create_subscription(
            LaserScan, SCAN_TOPIC, self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        self.frames.append(msg)


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main():
    rclpy.init()
    node = Probe()
    t0 = time.time()
    while len(node.frames) < N_SAMPLES and time.time() - t0 < TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.3)

    if not node.frames:
        print('❌ 没收到 /scan')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    msg = node.frames[-1]
    n = len(msg.ranges)
    rmin = msg.range_min

    print('=' * 70)
    print('雷达全景  frame=%s  n=%d  inc=%.2f°  range=[%.2f, %.1f]'
          % (msg.header.frame_id, n, math.degrees(msg.angle_increment),
             msg.range_min, msg.range_max))
    print('=' * 70)

    # 收集所有有效点（诊断范围 3m）
    pts = []
    for i, r in enumerate(msg.ranges):
        if math.isnan(r) or math.isinf(r):
            continue
        if r < rmin or r > MAX_RANGE:
            continue
        pts.append((r, wrap(msg.angle_min + i * msg.angle_increment)))

    print('%.1fm 内有效点: %d / %d' % (MAX_RANGE, len(pts), n))
    print()

    # 按扇区统计
    print('%-16s %6s %8s %8s' % ('扇区', '点数', '最近', '角度'))
    print('-' * 70)
    for lo, hi in SECTORS:
        seg = [(r, a) for r, a in pts
               if lo <= math.degrees(a) < hi
               or (hi == 180 and math.degrees(a) == 180)]
        if seg:
            near = min(seg, key=lambda t: t[0])
            mark = ''
            if lo == -30 and hi == 0:
                mark = '  ← 左前'
            elif lo == 0 and hi == 30:
                mark = '  ← 右前'
            elif lo <= 0 <= hi:
                mark = '  ← 正前'
            print('%-16s %6d %7.2fm %+7.1f°%s'
                  % ('[%d°, %d°]' % (lo, hi), len(seg), near[0],
                     math.degrees(near[1]), mark))
        else:
            print('%-16s %6d %8s %8s' % ('[%d°, %d°]' % (lo, hi), 0, '-', '-'))

    print('-' * 70)

    # 正前方 ±30° 详细
    front = [(r, a) for r, a in pts if abs(a) <= math.radians(30)]
    print()
    if front:
        print('正前方 ±30° 内 %d 个点:' % len(front))
        print('%-10s %-10s' % ('距离', '角度'))
        for r, a in sorted(front, key=lambda t: t[0])[:15]:
            print('  %.2fm    %+.1f°' % (r, math.degrees(a)))
    else:
        print('⚠️  正前方 ±30° 内一个点都没有 —— 那里是空的')

    # 最近的点在哪
    if pts:
        near = min(pts, key=lambda t: t[0])
        print()
        print('整个 3m 范围内最近的物体: %.2fm @ %+.1f°'
              % (near[0], math.degrees(near[1])))
        if abs(near[1]) > math.radians(30):
            print('  ⚠️ 它不在正前方扇区内，跟随节点看不到它')

    print('=' * 70)
    print('本脚本只读，未发布任何指令。')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
