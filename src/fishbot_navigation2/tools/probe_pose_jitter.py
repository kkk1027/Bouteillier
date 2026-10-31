#!/usr/bin/env python3
"""
位姿抖动长时监控 —— 只读，不发任何指令。

记录三路数据，用于区分抖动来源：
  1. /amcl_pose        AMCL 最终输出（map 系）
  2. TF map->odom      纯 AMCL 修正量
  3. TF odom->base_footprint   纯 EKF 里程计

判据：
  * 若 map->odom 跳而 odom->base 稳 → AMCL 在跳（粒子问题/重定位）
  * 若 odom->base 跳而 map->odom 稳 → EKF/编码器问题
  * 两者都跳 → 上游数据问题

用法：
    python3 probe_pose_jitter.py            # 默认 240 秒
    python3 probe_pose_jitter.py 60         # 60 秒
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


class PoseJitterProbe(Node):
    def __init__(self):
        super().__init__('pose_jitter_probe')

        self.amcl = None          # (x, y, yaw, sig_xy, sig_yaw, stamp)
        self.tf_ok = False

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self.on_amcl, 10)

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

        # 上一秒的参考值（用于算跳变）
        self.prev = None
        self.rows = []
        self.jumps = []

    # --------------------------------------------------------
    def on_amcl(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        c = msg.pose.covariance
        sx = math.sqrt(max(c[0], 0.0))
        sy = math.sqrt(max(c[7], 0.0))
        sa = math.sqrt(max(c[35], 0.0))
        self.amcl = (p.x, p.y, yaw_of(q), math.hypot(sx, sy), sa)

    # --------------------------------------------------------
    def _tf(self, target, source):
        """返回 (x, y, yaw)，查不到返回 None。用最新可用变换。"""
        try:
            t = self.buf.lookup_transform(
                target, source, rclpy.time.Time())
        except Exception:                       # noqa: BLE001
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, yaw_of(t.transform.rotation))

    # --------------------------------------------------------
    def sample(self):
        now = time.strftime('%H:%M:%S')

        a = self.amcl
        mo = self._tf('map', 'odom')
        ob = self._tf('odom', 'base_footprint')

        if a is None:
            print('%s  ⏳ 等 /amcl_pose' % now)
            return

        # 与上一秒比较，算跳变
        d_amcl = d_mo = d_ob = None
        flag = ''
        if self.prev is not None:
            pa, pmo, pob = self.prev
            d_amcl = math.hypot(a[0] - pa[0], a[1] - pa[1])
            if mo is not None and pmo is not None:
                d_mo = math.hypot(mo[0] - pmo[0], mo[1] - pmo[1])
            if ob is not None and pob is not None:
                d_ob = math.hypot(ob[0] - pob[0], ob[1] - pob[1])

            # 阈值：正常车静止时 <1cm；有人为挪动会大一些
            if d_amcl > 0.15:
                src = []
                if d_mo is not None and d_mo > 0.10:
                    src.append('AMCL')
                if d_ob is not None and d_ob > 0.10:
                    src.append('EKF')
                flag = '  <<< 跳变 %.2fm  来源=%s' % (
                    d_amcl, '+'.join(src) or '未知')
                self.jumps.append((now, d_amcl,
                                   d_mo or 0.0, d_ob or 0.0,
                                   '+'.join(src) or '未知'))

        self.prev = (a, mo, ob)

        print('%s  pose=(%6.3f,%6.3f) yaw=%7.2f° '
              'σxy=%.3f σyaw=%.3f | map->odom=%s | odom->base=%s%s'
              % (now, a[0], a[1], a[2], a[3], a[4],
                 ('(%6.3f,%6.3f)' % (mo[0], mo[1])) if mo else '  缺失  ',
                 ('(%6.3f,%6.3f)' % (ob[0], ob[1])) if ob else '  缺失  ',
                 flag))
        self.rows.append((now, a, mo, ob))

    # --------------------------------------------------------
    def summary(self, duration):
        print('\n' + '=' * 90)
        print('统计（%.0f 秒，%d 个采样点）' % (duration, len(self.rows)))
        print('=' * 90)

        if not self.rows:
            print('  没有采到 /amcl_pose')
            return

        # 位置跨度
        xs = [r[1][0] for r in self.rows]
        ys = [r[1][1] for r in self.rows]
        yaws = [r[1][2] for r in self.rows]
        print('  AMCL 位置跨度 : x %.3f~%.3f (%.3fm)  y %.3f~%.3f (%.3fm)'
              % (min(xs), max(xs), max(xs) - min(xs),
                 min(ys), max(ys), max(ys) - min(ys)))
        print('  AMCL 朝向跨度 : %.1f° ~ %.1f°  (%.1f°)'
              % (min(yaws), max(yaws), max(yaws) - min(yaws)))

        sig = [r[1][3] for r in self.rows]
        print('  σxy  范围     : %.3f ~ %.3f m' % (min(sig), max(sig)))

        mo_ok = sum(1 for r in self.rows if r[2] is not None)
        ob_ok = sum(1 for r in self.rows if r[3] is not None)
        print('  map->odom  可得: %d/%d' % (mo_ok, len(self.rows)))
        print('  odom->base 可得: %d/%d' % (ob_ok, len(self.rows)))

        print('\n  跳变次数（>0.15m/秒）: %d' % len(self.jumps))
        for ts, d, dmo, dob, src in self.jumps[:25]:
            print('    %s  AMCL %.2fm  (map->odom %.2fm, odom->base %.2fm)'
                  '  来源=%s' % (ts, d, dmo, dob, src))
        if len(self.jumps) > 25:
            print('    ... 另有 %d 次' % (len(self.jumps) - 25))

        # 结论
        print('\n  结论：')
        if not self.jumps:
            print('    未检测到显著跳变，位姿稳定')
        else:
            amcl_cnt = sum(1 for j in self.jumps if 'AMCL' in j[4])
            ekf_cnt = sum(1 for j in self.jumps if 'EKF' in j[4])
            unk_cnt = sum(1 for j in self.jumps if j[4] == '未知')
            print('    判定为 AMCL 引起: %d 次' % amcl_cnt)
            print('    判定为 EKF  引起: %d 次' % ekf_cnt)
            print('    来源不明（TF 缺失或都没动）: %d 次' % unk_cnt)
        print('=' * 90)


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0

    rclpy.init()
    node = PoseJitterProbe()

    print('=' * 90)
    print('位姿抖动监控  —— 只读，不发任何指令')
    print('  监控 %.0f 秒，每秒 1 个采样点' % duration)
    print('  请现在复现抖动（比如让 AI 导航 / 手动推车 / 点回起点）')
    print('=' * 90)

    t0 = time.time()
    next_sample = t0
    try:
        while time.time() - t0 < duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.time() >= next_sample:
                node.sample()
                next_sample += 1.0
    except KeyboardInterrupt:
        pass
    finally:
        node.summary(time.time() - t0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
