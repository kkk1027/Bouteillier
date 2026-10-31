#!/usr/bin/env python3
"""
合成数据验证 amcl_tracer 能正确捕获跳变与重定位事件。

不需要雷达/底盘 —— 本脚本自己扮演 AMCL + EKF + auto_localizer：
  1. 先稳定 3 秒（应判 CONVERGE）
  2. 模拟粒子塌缩（σ 压到极低，应判 COLLAPSE）
  3. 发 /initialpose 模拟重新撒粒子（应记 RESEED）
  4. 位姿瞬移 1.2m（应记 JUMP 且来源=AMCL）
  5. 再稳定

判据：跑完后检查 CSV 里这四类记录是否齐全。

用法：
    python3 test_tracer_synthetic.py
"""

import math
import os
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

TRACE = '/tmp/fishbot_amcl_trace_test.csv'


class FakeWorld(Node):
    def __init__(self):
        super().__init__('fake_world')
        self.amcl_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/odometry/filtered', 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.1, self.tick)      # 10Hz
        self.t = 0.0

    # --------------------------------------------------------
    def _pose(self, x, y, yaw, sxy, syaw):
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        c = [0.0] * 36
        c[0] = sxy * sxy
        c[7] = sxy * sxy
        c[35] = syaw * syaw
        m.pose.covariance = c
        return m

    def _tfmsg(self, parent, child, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        return t

    # --------------------------------------------------------
    def tick(self):
        self.t += 0.1
        s = self.t

        # 场景时间线
        if s < 3.0:                       # 稳定，σ 正常
            x, y, yaw, sxy, syaw = 1.0, 0.5, 0.1, 0.05, 0.08
        elif s < 5.0:                     # 粒子塌缩：σ 被压到极低
            x, y, yaw, sxy, syaw = 1.0, 0.5, 0.1, 0.0008, 0.0005
        elif s < 6.0:                     # 塌缩后即将重定位
            x, y, yaw, sxy, syaw = 1.0, 0.5, 0.1, 0.30, 0.40
        else:                             # 瞬移 1.2m（重定位结果错位）
            x, y, yaw, sxy, syaw = 2.2, 0.5, 0.1, 0.05, 0.08

        # 第 5 秒发一次 /initialpose（模拟 auto_localizer 重新撒粒子）
        if not hasattr(self, '_reseeded') and s >= 5.0:
            self._reseeded = True
            self.init_pub.publish(self._pose(1.0, 0.5, 0.1, 0.25, 0.0685))
            self.get_logger().info('>>> 发送 /initialpose（模拟重新撒粒子）')

        if abs(s - 6.0) < 0.06 and not hasattr(self, '_jumped'):
            self._jumped = True
            self.get_logger().info('>>> 位姿瞬移 1.2m（模拟重定位错位）')

        self.amcl_pub.publish(self._pose(x, y, yaw, sxy, syaw))

        # EKF 输出保持稳定（用于验证跳变来源归属到 AMCL 而非 EKF）
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_footprint'
        od.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(od)

        # TF：map->odom 承载 AMCL 修正，odom->base 恒定
        self.tf.sendTransform([
            self._tfmsg('map', 'odom', x, y, yaw),
            self._tfmsg('odom', 'base_footprint', 0.0, 0.0, 0.0),
        ])

        sc = LaserScan()
        sc.header.stamp = self.get_clock().now().to_msg()
        sc.header.frame_id = 'laser_link'
        sc.angle_min = -math.pi
        sc.angle_max = math.pi
        sc.angle_increment = math.radians(1.0)
        sc.range_min = 0.3
        sc.range_max = 12.0
        sc.ranges = [2.0] * 361
        self.scan_pub.publish(sc)


def main():
    if os.path.exists(TRACE):
        os.remove(TRACE)

    env = dict(os.environ)
    # __file__ = <ws>/src/fishbot_navigation2/tools/test_tracer_synthetic.py
    # 需要向上 4 层才是工作空间根（tools → fishbot_navigation2 → src → ws）
    work = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    tracer_log = open('/tmp/tracer_synth_node.log', 'w')
    tracer = subprocess.Popen(
        ['bash', '-c',
         'source /opt/ros/humble/setup.bash && '
         'source %s/install/setup.bash && '
         'exec ros2 run fishbot_navigation2 amcl_tracer.py --ros-args '
         '-p output_path:=%s -p heartbeat_sec:=0.5'
         % (work, TRACE)],
        env=env, stdout=tracer_log, stderr=subprocess.STDOUT)

    time.sleep(0.5)
    # ★ 不能固定 sleep：ros2 run + sourcing 要好几秒，
    #   tracer 还没起来就开始灌数据，前半段场景就白跑了。
    #   改成等记录文件出现（tracer 启动即在 __init__ 里建文件）。
    print('等 tracer 就绪...', end='', flush=True)
    deadline = time.time() + 30.0
    while time.time() < deadline and not os.path.exists(TRACE):
        time.sleep(0.2)
        print('.', end='', flush=True)
    if not os.path.exists(TRACE):
        print('\n❌ tracer 未能在 30s 内启动')
        tracer.kill()
        return 1
    print(' 就绪')

    # 再给它一点时间完成 ROS 发现（订阅建立）
    time.sleep(2.0)

    rclpy.init()
    node = FakeWorld()
    t0 = time.time()
    while time.time() - t0 < 9.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    time.sleep(0.8)
    tracer.terminate()
    try:
        tracer.wait(timeout=5)
    except subprocess.TimeoutExpired:
        tracer.kill()
    try:
        tracer_log.close()
    except Exception:                              # noqa: BLE001
        pass

    # ---------------- 检查结果 ----------------
    print('\n' + '=' * 66)
    print('验证结果')
    print('=' * 66)

    if not os.path.exists(TRACE):
        print('❌ 没有生成记录文件')
        return 1

    with open(TRACE, encoding='utf-8') as f:
        lines = f.readlines()

    text = ''.join(lines)
    checks = [
        ('记录了 COLLAPSE 状态', 'COLLAPSE' in text),
        ('记录了 RESEED 事件', 'RESEED' in text),
        ('记录了 JUMP 事件', 'JUMP' in text),
        ('跳变来源归属为 AMCL', 'JUMP[AMCL]' in text),
        ('记录了 CONVERGE 状态', 'CONVERGE' in text),
        ('有采样行（含坐标）', any(l.count(',') > 15 and l[0].isdigit()
                                  for l in lines)),
    ]
    ok = 0
    for name, cond in checks:
        print('  %s %s' % ('✅' if cond else '❌', name))
        ok += cond

    print('\n--- 事件时间线 ---')
    for l in lines:
        if l.startswith('#'):
            print('  ' + l.rstrip())

    print('\n--- 跳变行 ---')
    print('  ' + lines[1].rstrip())
    for l in lines:
        if 'JUMP' in l:
            print('  ' + l.rstrip())

    print('\n通过 %d/%d' % (ok, len(checks)))
    print('=' * 66)
    return 0 if ok == len(checks) else 1


if __name__ == '__main__':
    sys.exit(main())
