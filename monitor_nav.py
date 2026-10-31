#!/usr/bin/env python3
"""导航端到端实时监控 —— 观察优化后的定位质量。

设计目标
--------
把「网页端拉起导航 → 车端定位」整条链的关键指标实时打出来，
一次运行就能判断到底是哪一环没达标：

  odom→base_link   偏离起点多少（纯 odom 漂移，与 AMCL 无关）
  map→base_link    偏离起点多少（最终呈现给用户的位姿，含 AMCL）
  map→odom         AMCL 的修正量；它的刷新频率反映 nomotion 保活是否生效
  /amcl_pose       协方差（收敛判据）与位姿变化
  /odom |wz|       固件死区的直接证据

与 probe_stationary.py 的区别
-----------------------------
  probe_stationary.py  一次性采集、只读、输出汇总（静止基线用）
  monitor_nav.py       持续滚动打印，跑导航时挂在旁边看（本文件）

安全声明
--------
**只订阅，不发布任何消息**。不发 /cmd_vel、不发 /initialpose，
不会让小车移动。导航目标由网页端/你自己下发。

用法
----
    python3 monitor_nav.py              # 每 1 秒刷新一行
    python3 monitor_nav.py 2            # 每 2 秒一行
    python3 monitor_nav.py 1 log.csv    # 同时写 CSV

按下 Ctrl-C 会打印整段运行的汇总统计。
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

PERIOD = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
CSV_PATH = sys.argv[2] if len(sys.argv) > 2 else None


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    d = b - a
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class Monitor(Node):
    def __init__(self):
        super().__init__('monitor_nav')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

        best_effort = QoSProfile(depth=20,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self._on_odom, best_effort)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_amcl, 10)

        self.odom = None        # (x, y, yaw)
        self.odom_wz = 0.0
        self.odom_wz_max = 0.0
        self.amcl = None        # (x, y, yaw, sx, sy, syaw)

        self.ref = {}           # 起点参考：{'odom':(x,y), 'map':(x,y), 'mo':...}
        self.max_odom_dev = 0.0
        self.max_map_dev = 0.0
        self.max_map_yaw = 0.0
        self.mo_changes = 0     # map→odom 变化次数（判断刷新频率）
        self._last_mo = None
        self.map_odom = None
        self.rows = []
        self.t0 = time.time()
        self.create_timer(PERIOD, self._tick)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))
        self.odom_wz = msg.twist.twist.angular.z
        if abs(self.odom_wz) > self.odom_wz_max:
            self.odom_wz_max = abs(self.odom_wz)

    def _on_amcl(self, msg):
        c = msg.pose.covariance
        p = msg.pose.pose.position
        self.amcl = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation),
                     math.sqrt(max(0.0, c[0])),
                     math.sqrt(max(0.0, c[7])),
                     math.sqrt(max(0.0, c[35])))

    def _lookup(self, target, source):
        try:
            t = self.buf.lookup_transform(target, source, Time(),
                                          timeout=__import__(
                                              'rclpy').duration.Duration(
                                                  seconds=0.05))
            tr = t.transform.translation
            return (tr.x, tr.y, yaw_from_quat(t.transform.rotation))
        except (TransformException, Exception):   # noqa: BLE001
            return None

    def _tick(self):
        el = time.time() - self.t0
        ob = self._lookup('odom', 'base_link')
        mb = self._lookup('map', 'base_link')
        mo = self._lookup('map', 'odom')

        if mo is not None:
            self.map_odom = mo
            if self._last_mo is not None:
                dx = abs(mo[0] - self._last_mo[0])
                dy = abs(mo[1] - self._last_mo[1])
                da = abs(math.degrees(ang_diff(self._last_mo[2], mo[2])))
                if dx > 1e-4 or dy > 1e-4 or da > 0.01:
                    self.mo_changes += 1
            self._last_mo = mo

        # 起点参考用第一帧可用的位姿
        if ob and 'odom' not in self.ref:
            self.ref['odom'] = (ob[0], ob[1], ob[2])
        if mb and 'map' not in self.ref:
            self.ref['map'] = (mb[0], mb[1], mb[2])

        odom_dev = map_dev = map_yaw = None
        if ob and 'odom' in self.ref:
            d = math.hypot(ob[0] - self.ref['odom'][0],
                           ob[1] - self.ref['odom'][1])
            odom_dev = d
            self.max_odom_dev = max(self.max_odom_dev, d)
        if mb and 'map' in self.ref:
            d = math.hypot(mb[0] - self.ref['map'][0],
                           mb[1] - self.ref['map'][1])
            map_dev = d
            self.max_map_dev = max(self.max_map_dev, d)
            map_yaw = abs(math.degrees(ang_diff(self.ref['map'][2], mb[2])))
            self.max_map_yaw = max(self.max_map_yaw, map_yaw)

        # ---------------- 实时行 ----------------
        s = '[%5.0fs] ' % el
        s += 'odom_wz=%+.4f ' % self.odom_wz
        odom_txt = '  ---  ' if odom_dev is None else '%.3fm' % odom_dev
        map_txt = '  ---  ' if map_dev is None else '%.3fm' % map_dev
        yaw_txt = ' --- ' if map_yaw is None else '%.1f°' % map_yaw
        s += 'odom偏离=%s ' % odom_txt
        s += 'map偏离=%s ' % map_txt
        s += 'map_yaw=%s ' % yaw_txt
        if self.amcl:
            s += 'σ_xy=%.3f σ_yaw=%.1f°' % (
                max(self.amcl[3], self.amcl[4]),
                math.degrees(self.amcl[5]))
        else:
            s += 'σ=(等待AMCL)'
        s += ' mo刷新=%d' % self.mo_changes
        print(s, flush=True)

        self.rows.append((round(el, 1),
                          round(self.odom_wz, 5),
                          None if odom_dev is None else round(odom_dev, 4),
                          None if map_dev is None else round(map_dev, 4),
                          None if map_yaw is None else round(map_yaw, 2)))

    def summary(self):
        el = time.time() - self.t0
        print()
        print('=' * 70)
        print(' 监控汇总（%.0f 秒）' % el)
        print('=' * 70)
        print('  /odom |wz| 最大值      : %.6f rad/s   %s'
              % (self.odom_wz_max,
                 '✅ 死区生效' if self.odom_wz_max < 1e-6
                 else ('⚠️ 偏大' if self.odom_wz_max < 0.02
                       else '❌ 死区未生效')))
        print('  odom→base_link 最大偏离 : %.3f m' % self.max_odom_dev)
        print('  map →base_link 最大偏离 : %.3f m' % self.max_map_dev)
        print('  map →base_link 最大转角 : %.1f°' % self.max_map_yaw)
        print('  map→odom 刷新次数       : %d（%.2f 次/秒）'
              % (self.mo_changes, self.mo_changes / el if el else 0))
        if self.map_odom:
            print('  当前 map→odom          : (%.3f, %.3f, %.1f°)'
                  % (self.map_odom[0], self.map_odom[1],
                     math.degrees(self.map_odom[2])))
        print()
        if el > 10:
            rate = self.mo_changes / el
            if rate >= 0.3:
                print('  ✅ map→odom 刷新正常（nomotion 保活生效，'
                      '静止时 AMCL 仍在修正）')
            elif rate > 0:
                print('  ⚠️  map→odom 刷新偏慢（%.2f 次/秒），'
                      '检查 nomotion_update_period_after_converge_sec' % rate)
            else:
                print('  ❌ map→odom 完全没更新 —— 要么 AMCL 未收敛，'
                      '要么 auto_localizer 已停止请求')
        print('=' * 70)
        if CSV_PATH:
            try:
                with open(CSV_PATH, 'w') as f:
                    f.write('t_sec,odom_wz,odom_dev_m,map_dev_m,map_yaw_deg\n')
                    for r in self.rows:
                        f.write(','.join('' if v is None else str(v)
                                         for v in r) + '\n')
                print(' CSV 已写入 %s' % CSV_PATH)
            except OSError as e:
                print(' 写 CSV 失败: %s' % e)


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.summary()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
