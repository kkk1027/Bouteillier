#!/usr/bin/env python3
"""静止漂移探针 —— 量化验证 odom 死区修复效果。

背景
----
固件 Kinematics.cpp 原先对编码器速度无条件积分，±1~2 tick 量化噪声
被积成 0.025~0.05 rad/s 的虚假角速度，导致小车完全静止时 odom 的
yaw 也持续漂。2026-10-31 已在固件里加自适应死区。

本脚本在小车静止时同时采集：
  /odom             原始编码器里程计（验证固件死区）
  /imu_raw          陀螺原始读数（看真实静止噪声水平）
  /amcl_pose        若导航栈在跑，看协方差是否收敛
  /odometry/filtered 若 EKF 在跑，看融合后的漂移
并把 odom→base_link 的位姿按时间采样，算出「每秒漂多少」。

关键判据
--------
  odom |wz| 最大值      应恒为 0.0（死区生效）；若仍 >0.02 说明固件未更新
  odom yaw 累计漂移     <0.5°/min 为合格
  filtered yaw 累计漂移 <1.0°/min 为合格

安全声明
--------
**只订阅，不发布任何消息**。/cmd_vel、/initialpose 全程不碰，小车不会动。

用法
----
    python3 probe_stationary.py            # 采集 60 秒
    python3 probe_stationary.py 120        # 采集 120 秒
输出同时写入 stdout 与 /tmp/probe_stationary.csv
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
SAMPLE_DT = 0.5        # 位姿采样间隔（秒）
CSV_PATH = '/tmp/probe_stationary.csv'


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    """最小角差 b-a，归一到 [-pi, pi]。"""
    d = b - a
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class Probe(Node):
    def __init__(self):
        super().__init__('probe_stationary')
        best_effort = QoSProfile(depth=20,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self._on_odom, best_effort)
        self.create_subscription(Imu, '/imu_raw', self._on_imu, best_effort)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self._on_filtered, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_amcl, 10)

        # ---- 统计量 ----
        self.odom_n = 0
        self.odom_wz_absmax = 0.0
        self.odom_wz_nonzero = 0          # |wz| > 1e-6 的样本数
        self.odom_yaw0 = None
        self.odom_yaw_last = None
        self.odom_xy0 = None
        self.odom_xy_last = None

        self.imu_n = 0
        self.imu_wz_absmax = 0.0
        self.imu_wz_sum = 0.0

        self.filt_n = 0
        self.filt_yaw0 = None
        self.filt_yaw_last = None

        self.amcl_n = 0
        self.amcl_cov_last = None
        self.amcl_xy0 = None
        self.amcl_xy_last = None

        self.samples = []                 # (t, odom_yaw, odom_x, odom_y, ...)
        self.t0 = time.time()
        self.create_timer(SAMPLE_DT, self._sample)

    # ---------------- 回调：只记录，不做任何判断 ----------------
    def _on_odom(self, msg):
        self.odom_n += 1
        wz = msg.twist.twist.angular.z
        a = abs(wz)
        if a > self.odom_wz_absmax:
            self.odom_wz_absmax = a
        if a > 1e-6:
            self.odom_wz_nonzero += 1
        y = yaw_from_quat(msg.pose.pose.orientation)
        if self.odom_yaw0 is None:
            self.odom_yaw0 = y
        self.odom_yaw_last = y
        p = msg.pose.pose.position
        if self.odom_xy0 is None:
            self.odom_xy0 = (p.x, p.y)
        self.odom_xy_last = (p.x, p.y)

    def _on_imu(self, msg):
        self.imu_n += 1
        wz = msg.angular_velocity.z
        self.imu_wz_sum += wz
        if abs(wz) > self.imu_wz_absmax:
            self.imu_wz_absmax = abs(wz)

    def _on_filtered(self, msg):
        self.filt_n += 1
        y = yaw_from_quat(msg.pose.pose.orientation)
        if self.filt_yaw0 is None:
            self.filt_yaw0 = y
        self.filt_yaw_last = y

    def _on_amcl(self, msg):
        self.amcl_n += 1
        c = msg.pose.covariance
        self.amcl_cov_last = (math.sqrt(max(0.0, c[0])),
                              math.sqrt(max(0.0, c[7])),
                              math.sqrt(max(0.0, c[35])))
        p = msg.pose.pose.position
        if self.amcl_xy0 is None:
            self.amcl_xy0 = (p.x, p.y)
        self.amcl_xy_last = (p.x, p.y)

    # ---------------- 周期采样 ----------------
    def _sample(self):
        if time.time() - self.t0 > DURATION:
            raise SystemExit(0)
        self.samples.append((
            round(time.time() - self.t0, 2),
            self.odom_yaw_last,
            self.filt_yaw_last,
            self.amcl_xy_last[0] if self.amcl_xy_last else None,
        ))


def main():
    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass

    el = time.time() - node.t0
    print()
    print('=' * 66)
    print(' 静止探针结果（采集 %.0f 秒）' % el)
    print('=' * 66)

    # ---------- /odom（固件死区效果）----------
    print()
    print('【1】/odom 原始编码器里程计（验证固件死区）')
    print('    样本数            : %d（%.1f Hz）'
          % (node.odom_n, node.odom_n / el if el else 0))
    print('    |wz| 最大值       : %.6f rad/s' % node.odom_wz_absmax)
    print('    |wz|>1e-6 的样本  : %d / %d'
          % (node.odom_wz_nonzero, node.odom_n))
    if node.odom_yaw0 is not None:
        dyaw = math.degrees(ang_diff(node.odom_yaw0, node.odom_yaw_last))
        dxy = 0.0
        if node.odom_xy0 and node.odom_xy_last:
            dxy = math.hypot(node.odom_xy_last[0] - node.odom_xy0[0],
                             node.odom_xy_last[1] - node.odom_xy0[1])
        print('    yaw 累计漂移      : %+.3f°  (%.3f°/min)'
              % (dyaw, dyaw / el * 60 if el else 0))
        print('    xy  累计漂移      : %.4f m' % dxy)
    if node.odom_wz_absmax < 1e-6:
        print('    ✅ 判定：死区完全生效，静止时 wz 恒为 0')
    elif node.odom_wz_absmax < 0.02:
        print('    ⚠️  判定：wz 很小但非 0，死区部分生效')
    else:
        print('    ❌ 判定：wz 仍达 %.3f rad/s，固件死区**未生效**'
              % node.odom_wz_absmax)

    # ---------- /imu_raw（真实静止噪声基线）----------
    print()
    print('【2】/imu_raw 陀螺原始读数（真实静止噪声）')
    if node.imu_n:
        print('    样本数            : %d' % node.imu_n)
        print('    |wz| 最大值       : %.6f rad/s' % node.imu_wz_absmax)
        print('    wz 均值(零偏)     : %+.6f rad/s  (%+.3f °/s)'
              % (node.imu_wz_sum / node.imu_n,
                 node.imu_wz_sum / node.imu_n * 57.2958))
        bias = node.imu_wz_sum / node.imu_n
        if abs(bias) < 0.005:
            print('    ✅ 判定：零偏已补偿得很干净')
        else:
            print('    ⚠️  判定：零偏 %.4f rad/s 偏大，检查 imu_bias_remover'
                  % bias)
    else:
        print('    ⚠️  未收到 /imu_raw（ESP32 可能未连接）')

    # ---------- /odometry/filtered ----------
    print()
    print('【3】/odometry/filtered（EKF 融合结果）')
    if node.filt_n and node.filt_yaw0 is not None:
        dyaw = math.degrees(ang_diff(node.filt_yaw0, node.filt_yaw_last))
        print('    样本数            : %d' % node.filt_n)
        print('    yaw 累计漂移      : %+.3f°  (%.3f°/min)'
              % (dyaw, dyaw / el * 60 if el else 0))
        if abs(dyaw / el * 60 if el else 0) < 1.0:
            print('    ✅ 判定：融合后静止漂移 < 1°/min，合格')
        else:
            print('    ❌ 判定：融合后仍漂 %.2f°/min' % (dyaw / el * 60))
    else:
        print('    （无数据 —— EKF 只在 navigation2 / manual_mapping 中启动）')

    # ---------- /amcl_pose ----------
    print()
    print('【4】/amcl_pose（AMCL 定位）')
    if node.amcl_n and node.amcl_cov_last:
        sx, sy, syaw = node.amcl_cov_last
        print('    样本数            : %d' % node.amcl_n)
        print('    最新协方差        : σ_xy=%.3fm σ_yaw=%.2f°'
              % (max(sx, sy), math.degrees(syaw)))
        if node.amcl_xy0 and node.amcl_xy_last:
            d = math.hypot(node.amcl_xy_last[0] - node.amcl_xy0[0],
                           node.amcl_xy_last[1] - node.amcl_xy0[1])
            print('    位置累计变化      : %.4f m' % d)
            if d < 0.05:
                print('    ✅ 判定：静止时 AMCL 位姿稳定')
            else:
                print('    ❌ 判定：静止时 AMCL 位姿漂了 %.3f m' % d)
    else:
        print('    （无数据 —— 需要导航栈在运行且已定位）')

    # ---------- 写 CSV ----------
    try:
        with open(CSV_PATH, 'w') as f:
            f.write('t_sec,odom_yaw,filtered_yaw,amcl_x\n')
            for t, oy, fy, ax in node.samples:
                f.write('%s,%s,%s,%s\n' % (
                    t,
                    '' if oy is None else round(oy, 6),
                    '' if fy is None else round(fy, 6),
                    '' if ax is None else round(ax, 4)))
        print()
        print('时间序列已写入 %s（可用 Excel/gnuplot 画曲线）' % CSV_PATH)
    except OSError as exc:
        print('写 CSV 失败: %s' % exc)

    print()
    print('=' * 66)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
