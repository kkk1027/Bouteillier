#!/usr/bin/env python3
"""
lidar_follow.py
单线激光雷达目标跟随节点。

================================================================
控制目标
================================================================

让小车与「前方最近障碍物」保持约 50cm，并对准它。

    ±30° 扇区
        ╲   │   ╱
         ╲  │  ╱
          ╲ │ ╱
        ┌───────┐
        │  车   │  ← 雷达在车体正中心
        └───────┘

处理流程（每次扫描，5Hz）：

    LaserScan
       │
       ├─ 1. 全向提取有效点（几何过滤：NaN/Inf/量程）
       │
       ├─ 2. 聚类
       │      全向簇 → 前后方安全检查
       │      前方簇 → 目标选择
       │
       ├─ 3. 目标选择（两段式：首次锁定 40cm，跟踪放宽到 1.2m）
       │
       ├─ 4. 卡尔曼滤波 (d, θ) 各一个 CV 模型
       │
       └─ 5. 三态状态机 + PD 控制 → /cmd_vel

================================================================
三态滞回（核心防抖）
================================================================

    距离轴 ────────────────────────────────────────▶

      0.30   0.50      0.55        0.65        1.20
       │  RETREAT │  HOLD  │  APPROACH  │
       │          │        │            │
       │     后退到│0.55 停 │0.55 前进到停│
       │          │        │            │
       └ 进入后退: d<0.50   └ 进入前进: d>0.65

    中间 0.55~0.65 是死区，目标在此范围内微动时小车完全不动。

    进入和退出用不同阈值，形成 10cm 滞回带。
    如果都用 0.50，目标在 0.50 附近抖动时会
    RETREAT → HOLD → RETREAT → HOLD 疯狂翻转，电机来回抽。

================================================================
安全兜底
================================================================

  * 雷达超时 0.6s 无声        → 停车
  * 目标丢失                  → 立即停车（不原地转找）
  * 后退前查后方 40cm         → 有障碍就不退
  * 前进时前方 32cm 有更近的  → 刹车
  * 节点退出                  → 必定连发零速度
  * ESP32 3 秒无 Twist        → 自动停车（最后防线）

================================================================
模块划分
================================================================

    geometry.py         点云提取/聚类/选择（纯函数，无 ROS 依赖）
    tracker.py          卡尔曼与目标跟踪器
    mutual_exclusion.py 与导航互斥的锁文件
    lidar_follow.py     本文件：ROS 节点装配 + 状态机 + 控制输出
"""

import math
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String

from fishbot_lidar_follow import geometry
from fishbot_lidar_follow import mutual_exclusion
from fishbot_lidar_follow.tracker import TargetTracker


# ============================================================
# 状态定义
# ============================================================
ST_IDLE = 'IDLE'          # 等待启动
ST_SEARCH = 'SEARCH'      # 已启动但还没锁定目标
ST_APPROACH = 'APPROACH'  # 目标太远，前进
ST_HOLD = 'HOLD'          # 距离合适，停住
ST_RETREAT = 'RETREAT'    # 目标太近，后退

STATE_CODES = {
    ST_IDLE: 0, ST_SEARCH: 1, ST_APPROACH: 2,
    ST_HOLD: 3, ST_RETREAT: 4,
}

# 调试话题的布局：[距离, 角度, 距离变化率, 角度变化率, 簇数量, 状态码]
DEBUG_FIELDS = ('dist', 'angle', 'dist_rate', 'angle_rate',
                'n_clusters', 'state')


# ============================================================
# 参数默认值
# ------------------------------------------------------------
# 都在这里集中声明，方便和 config/lidar_follow.yaml 对照
# ============================================================
DEFAULT_PARAMS = {
    # 输入输出
    'scan_topic': '/scan',
    'cmd_vel_topic': '/cmd_vel',
    'control_rate': 20.0,

    # 目标识别
    # lock_max_range 实机标定：雷达在车体正中心，用户量的「前方40cm」
    # 若从车头算，雷达读数会多出车体半径。实测物体在 0.44m 时点云
    # 24~28 点、单簇、极稳，但 0.40 差 4cm 锁不上 → 调到 0.50。
    # 上限：不能超过 retreat_enter(0.50)，否则锁定后立刻触发后退。
    'lock_max_range': 0.50,
    'track_max_range': 1.20,
    'sector_half_angle_deg': 30.0,
    'cluster_angle_gap_deg': 2.5,
    'cluster_range_gap': 0.15,
    # 点数少于这个值的簇直接忽略。
    # 实测：真实雷达里 1~3 点的簇几乎都是噪声（柜子边缘、反光），
    # 但距离可能只有 0.8m，不滤掉就会被当成目标“追过去”。
    'min_cluster_points': 5,
    # 已锁定时，新目标距离与当前相差超过此值 → 判为换了物体，
    # 当作本帧没目标（交给丢帧逻辑），而不是立刻切过去。
    # 实测：目标被推进 30cm（雷达量程下限）时近点全部消失，
    # 剩下的远簇只差 0.54m，不拦住就会切换目标并前进。
    'max_target_jump': 0.30,

    # 距离死区
    'approach_enter': 0.65,
    'approach_exit': 0.55,
    'retreat_enter': 0.50,
    'retreat_exit': 0.55,
    'desired_distance': 0.575,
    'distance_deadband': 0.02,

    # 角度
    'angle_deadband_deg': 2.0,
    'align_before_move_deg': 15.0,

    # 速度
    'max_linear': 0.22,
    'max_reverse': 0.12,
    'max_angular': 0.80,
    'kp_linear': 1.10,
    'kd_linear': 0.18,
    'kp_angular': 1.60,
    'kd_angular': 0.12,
    'cmd_linear_deadband': 0.015,
    'cmd_angular_deadband': 0.020,
    'linear_accel_limit': 0.80,
    'angular_accel_limit': 2.50,

    # 卡尔曼
    'kf_process_noise_pos': 0.0015,
    'kf_process_noise_vel': 0.060,
    'kf_measure_noise_dist': 0.010,
    'kf_measure_noise_angle': 0.012,

    # 安全
    'rear_check_range': 0.40,
    'rear_sector_half_angle_deg': 40.0,
    'front_guard_range': 0.32,
    'scan_timeout': 0.60,
    'lost_frame_threshold': 3,

    # 互斥
    'stop_nav_on_start': True,

    # 调试
    'publish_debug': True,
    'debug_topic': '/lidar_follow/debug',
    'state_topic': '/lidar_follow/state',
}


class LidarFollowNode(Node):
    """雷达跟随节点。"""

    def __init__(self):
        super().__init__('lidar_follow_node')
        self._declare_params()
        self._read_params()
        self._validate_params()

        # ---- 运行状态 ----
        self.state = ST_SEARCH
        self.locked = False
        self.tracker = TargetTracker(
            self.kf_q_pos, self.kf_q_vel,
            self.kf_r_dist, self.kf_r_angle)

        self.target_valid = False
        self.target_dist = 0.0
        self.target_angle = 0.0
        self.target_dist_rate = 0.0
        self.target_angle_rate = 0.0

        self.last_scan_time = 0.0
        self.last_clusters = []      # 前方簇，给目标选择
        self.all_clusters = []       # 全向簇，给安全检查

        # 速度状态（加速度限幅用）
        self.last_lin = 0.0
        self.last_ang = 0.0
        self.last_ctrl_time = time.monotonic()

        # 日志去重
        self._last_state = None
        self._last_reason = None

        # 统计
        self.stat_locked = 0
        self.stat_lost = 0

        # ---- 通信 ----
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)

        self.debug_pub = None
        if self.publish_debug:
            self.debug_pub = self.create_publisher(
                Float32MultiArray, self.debug_topic, 10)

        # ---- 控制循环 ----
        self.timer = self.create_timer(1.0 / self.control_rate, self.on_ctrl)

        # ---- 互斥 ----
        if self.stop_nav_on_start:
            self._stop_existing_nav()
        mutual_exclusion.write_lock(os.getpid())
        self.get_logger().info('已写跟随锁 %s' % mutual_exclusion.LOCK_FILE)

        self._log_startup()

    # --------------------------------------------------------
    # 参数
    # --------------------------------------------------------
    def _declare_params(self):
        for k, v in DEFAULT_PARAMS.items():
            self.declare_parameter(k, v)

    def _read_params(self):
        g = lambda k: self.get_parameter(k).value  # noqa: E731

        self.scan_topic = g('scan_topic')
        self.cmd_topic = g('cmd_vel_topic')
        self.control_rate = float(g('control_rate'))
        self.state_topic = g('state_topic')

        self.lock_max_range = float(g('lock_max_range'))
        self.track_max_range = float(g('track_max_range'))
        self.sector_rad = math.radians(float(g('sector_half_angle_deg')))
        self.cluster_angle_gap = math.radians(
            float(g('cluster_angle_gap_deg')))
        self.cluster_range_gap = float(g('cluster_range_gap'))
        self.min_cluster_points = int(g('min_cluster_points'))
        self.max_target_jump = float(g('max_target_jump'))

        self.approach_enter = float(g('approach_enter'))
        self.approach_exit = float(g('approach_exit'))
        self.retreat_enter = float(g('retreat_enter'))
        self.retreat_exit = float(g('retreat_exit'))
        self.desired_distance = float(g('desired_distance'))
        self.distance_deadband = float(g('distance_deadband'))

        self.angle_deadband = math.radians(float(g('angle_deadband_deg')))
        self.align_before_move = math.radians(
            float(g('align_before_move_deg')))

        self.max_linear = float(g('max_linear'))
        self.max_reverse = float(g('max_reverse'))
        self.max_angular = float(g('max_angular'))
        self.kp_linear = float(g('kp_linear'))
        self.kd_linear = float(g('kd_linear'))
        self.kp_angular = float(g('kp_angular'))
        self.kd_angular = float(g('kd_angular'))
        self.cmd_lin_db = float(g('cmd_linear_deadband'))
        self.cmd_ang_db = float(g('cmd_angular_deadband'))
        self.lin_acc = float(g('linear_accel_limit'))
        self.ang_acc = float(g('angular_accel_limit'))

        self.kf_q_pos = float(g('kf_process_noise_pos'))
        self.kf_q_vel = float(g('kf_process_noise_vel'))
        self.kf_r_dist = float(g('kf_measure_noise_dist'))
        self.kf_r_angle = float(g('kf_measure_noise_angle'))

        self.rear_range = float(g('rear_check_range'))
        self.rear_rad = math.radians(float(g('rear_sector_half_angle_deg')))
        self.front_guard = float(g('front_guard_range'))
        self.scan_timeout = float(g('scan_timeout'))
        self.lost_threshold = int(g('lost_frame_threshold'))

        self.stop_nav_on_start = bool(g('stop_nav_on_start'))
        self.publish_debug = bool(g('publish_debug'))
        self.debug_topic = g('debug_topic')

    def _validate_params(self):
        """参数自检。死区顺序错了会让车来回抽，必须当场报出来。"""
        ok = (self.retreat_enter < self.retreat_exit
              <= self.approach_exit < self.approach_enter)
        if not ok:
            self.get_logger().error(
                '距离死区参数顺序错误！应满足 '
                'retreat_enter(%s) < retreat_exit(%s) <= '
                'approach_exit(%s) < approach_enter(%s)'
                % (self.retreat_enter, self.retreat_exit,
                   self.approach_exit, self.approach_enter))

        if self.lock_max_range >= self.track_max_range:
            self.get_logger().warn(
                'lock_max_range(%.2f) >= track_max_range(%.2f)，'
                '锁定后不会有更宽的跟踪范围，可能一锁定就丢失'
                % (self.lock_max_range, self.track_max_range))

        # 锁定门槛超过后退进入线 → 一锁定就满足「太近了」，
        # 车会立刻开始后退，用户会觉得「刚放好物品车就倒退」。
        if self.lock_max_range > self.retreat_enter:
            self.get_logger().warn(
                'lock_max_range(%.2f) > retreat_enter(%.2f)，'
                '目标在 %.2f~%.2f 之间被锁定时会立刻后退。'
                '这通常不是想要的，建议 lock_max_range <= retreat_enter'
                % (self.lock_max_range, self.retreat_enter,
                   self.retreat_enter, self.lock_max_range))

    def _log_startup(self):
        self.get_logger().info('=' * 58)
        self.get_logger().info('雷达跟随节点已启动')
        self.get_logger().info('  扫描话题 : %s' % self.scan_topic)
        self.get_logger().info('  输出话题 : %s' % self.cmd_topic)
        self.get_logger().info('  锁定范围 : 首次 %.2fm / 跟踪 %.2fm'
                               % (self.lock_max_range, self.track_max_range))
        self.get_logger().info(
            '  距离死区 : 后退<%.2f  停 %.2f~%.2f  前进>%.2f'
            % (self.retreat_enter, self.approach_exit,
               self.approach_enter, self.approach_enter))
        self.get_logger().info('  扇区     : ±%.0f°  (后方 ±%.0f°)'
                               % (math.degrees(self.sector_rad),
                                  math.degrees(self.rear_rad)))
        self.get_logger().info('=' * 58)

    # --------------------------------------------------------
    # 与导航互斥
    # --------------------------------------------------------
    def _stop_existing_nav(self):
        """杀掉 nav2 / navigation2.launch 相关进程。

        只按明确的 launch 标记匹配，绝不用 'python3' 这类宽泛模式，
        否则会误杀本节点和 web_controller。
        """
        marks = ['navigation2.launch', 'nav2_bringup', 'auto_localizer']
        killed = 0
        for mark in marks:
            try:
                out = subprocess.run(
                    ['pgrep', '-f', mark],
                    capture_output=True, text=True, timeout=3)
                for line in out.stdout.split():
                    try:
                        pid = int(line)
                    except ValueError:
                        continue
                    if pid == os.getpid():
                        continue
                    try:
                        os.kill(pid, signal.SIGINT)
                        killed += 1
                    except Exception:               # noqa: BLE001
                        pass
            except Exception:                       # noqa: BLE001
                pass
        if killed:
            self.get_logger().info(
                '已向 %d 个导航进程发送 SIGINT，等待退出' % killed)
            time.sleep(1.5)

    # --------------------------------------------------------
    # 扫描回调
    # --------------------------------------------------------
    def on_scan(self, msg):
        stamp = time.monotonic()
        self.last_scan_time = stamp

        # 全向提取（安全检查要看后方，不能在这里限扇区）
        all_pts = geometry.extract_points(msg, self.track_max_range * 1.15)
        self.all_clusters = geometry.cluster_points(
            all_pts, self.cluster_angle_gap, self.cluster_range_gap)

        front_pts = geometry.sector_filter(all_pts, self.sector_rad)
        self.last_clusters = geometry.cluster_points(
            front_pts, self.cluster_angle_gap, self.cluster_range_gap)

        pick = geometry.select_target(
            self.last_clusters, self.locked,
            self.target_dist if self.locked and self.target_valid else 0.0,
            self.target_angle if self.locked and self.target_valid else 0.0,
            self.lock_max_range, self.track_max_range,
            min_points=self.min_cluster_points,
            max_jump=self.max_target_jump)

        if pick is None:
            self._handle_no_target(stamp)
            return

        dist, angle, npts = pick

        # 首次锁定判定
        if not self.locked:
            if dist > self.lock_max_range:
                self.target_valid = False
                return
            self.locked = True
            self.stat_locked += 1
            self.tracker.reset()
            self.get_logger().info(
                '🔒 已锁定目标：%.2fm / %.1f°（%d 点），开始跟随'
                % (dist, math.degrees(angle), npts))

        fd, fa, dv, av = self.tracker.observe(dist, angle, stamp)
        self.target_dist = fd
        self.target_angle = fa
        self.target_dist_rate = dv
        self.target_angle_rate = av
        self.target_valid = True

    def _handle_no_target(self, stamp):
        if not self.locked:
            self.target_valid = False
            return

        pd, pa = self.tracker.predict_only(stamp)
        if self.tracker.lost_frames >= self.lost_threshold:
            if self.target_valid:
                self.get_logger().warn(
                    '目标丢失（连续 %d 帧），停车等待'
                    % self.tracker.lost_frames)
                self.stat_lost += 1
            self.target_valid = False
        else:
            # 短暂丢帧：用预测值维持，避免单帧抖动就急刹
            self.target_dist = pd
            self.target_angle = pa
            self.target_valid = True

    # --------------------------------------------------------
    # 状态机（三态滞回）
    # --------------------------------------------------------
    def update_state(self, d):
        s = self.state

        if s in (ST_IDLE, ST_SEARCH):
            # 一有有效目标就进 HOLD，下个周期再按距离决定
            return ST_HOLD

        if s == ST_HOLD:
            if d > self.approach_enter:
                return ST_APPROACH
            if d < self.retreat_enter:
                return ST_RETREAT
            return ST_HOLD

        if s == ST_APPROACH:
            # 前进到 approach_exit 就停，不会一直怼上去
            return ST_HOLD if d <= self.approach_exit else ST_APPROACH

        if s == ST_RETREAT:
            # 退到 retreat_exit 就停，不会一直退
            return ST_HOLD if d >= self.retreat_exit else ST_RETREAT

        return ST_HOLD

    # --------------------------------------------------------
    # 控制循环
    # --------------------------------------------------------
    def on_ctrl(self):
        now = time.monotonic()
        dt = max(1e-3, now - self.last_ctrl_time)
        self.last_ctrl_time = now

        lin, ang, reason = self._compute_cmd(now)

        # 输出死区
        if abs(lin) < self.cmd_lin_db:
            lin = 0.0
        if abs(ang) < self.cmd_ang_db:
            ang = 0.0

        # 加速度限幅
        lin = _slew(self.last_lin, lin, self.lin_acc * dt)
        ang = _slew(self.last_ang, ang, self.ang_acc * dt)
        self.last_lin, self.last_ang = lin, ang

        self.publish_cmd(lin, ang)
        self.publish_state(reason)

    def _compute_cmd(self, now):
        """返回 (线速度, 角速度, 原因说明)。"""
        # ---- 安全 1：雷达失联 ----
        if self.last_scan_time <= 0.0:
            return 0.0, 0.0, '等待首帧扫描'

        if (now - self.last_scan_time) > self.scan_timeout:
            self.state = ST_SEARCH
            self.locked = False
            self.target_valid = False
            return 0.0, 0.0, '雷达超时 %.2fs' % (now - self.last_scan_time)

        # ---- 安全 2：目标丢失 ----
        if not self.target_valid:
            self.state = ST_SEARCH
            return 0.0, 0.0, '目标丢失，停车'

        # ---- 正常跟随 ----
        d = self.target_dist
        a = self.target_angle
        self.state = self.update_state(d)

        e_d = d - self.desired_distance
        if abs(e_d) < self.distance_deadband:
            e_d = 0.0
        v_lin = self.kp_linear * e_d + self.kd_linear * self.target_dist_rate

        e_a = a
        if abs(e_a) < self.angle_deadband:
            e_a = 0.0
        v_ang = self.kp_angular * e_a + self.kd_angular * self.target_angle_rate

        if self.state == ST_APPROACH:
            lin = min(v_lin, self.max_linear)
        elif self.state == ST_RETREAT:
            lin = max(v_lin, -self.max_reverse)
        else:
            lin = 0.0
            if abs(e_a) < self.angle_deadband:
                v_ang = 0.0      # HOLD 且已对准 → 完全不动

        # 角度偏太大时先转正，别一边大角度转一边冲
        if abs(e_a) > self.align_before_move and d < 0.90:
            lin = 0.0
            reason = '先对准 %.0f°' % math.degrees(e_a)
        else:
            reason = '%s d=%.2f a=%.1f°' % (
                self.state, d, math.degrees(a))

        # ---- 安全 3：后方有障碍不许后退 ----
        if lin < 0.0 and not geometry.rear_is_clear(
                self.all_clusters, self.rear_rad, self.rear_range):
            lin = 0.0
            reason += ' | 后方%.1fm内有障碍，不后退' % self.rear_range

        # ---- 安全 4：前方有比目标更近的东西 ----
        if lin > 0.0 and d > self.front_guard:
            nearest = geometry.front_nearest(self.last_clusters)
            if nearest is not None and nearest < self.front_guard:
                lin = 0.0
                reason += ' | 前方%.2fm有障碍，刹车' % nearest

        ang = max(-self.max_angular, min(self.max_angular, v_ang))
        return lin, ang, reason

    # --------------------------------------------------------
    # 发布
    # --------------------------------------------------------
    def publish_cmd(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

        if self.debug_pub is not None:
            m = Float32MultiArray()
            m.data = [
                float(self.target_dist),
                float(self.target_angle),
                float(self.target_dist_rate),
                float(self.target_angle_rate),
                float(len(self.last_clusters)),
                float(STATE_CODES.get(self.state, 9)),
            ]
            self.debug_pub.publish(m)

    def publish_state(self, reason):
        s = String()
        s.data = '%s|%.2f|%.1f|%s' % (
            self.state, self.target_dist, math.degrees(self.target_angle),
            'LOCK' if self.locked else 'NOLOCK')
        self.state_pub.publish(s)

        # 状态变化才打 info，避免 20Hz 刷屏
        if reason != self._last_reason:
            self._last_reason = reason
            if self.state != self._last_state:
                self.get_logger().info(
                    '状态 → %s  (%s)' % (self.state, reason))
            else:
                self.get_logger().debug('  %s' % reason)
            self._last_state = self.state

    # --------------------------------------------------------
    # 退出
    # --------------------------------------------------------
    def shutdown(self):
        """安全停车并清理。"""
        for _ in range(3):
            self.publish_cmd(0.0, 0.0)
            time.sleep(0.02)
        mutual_exclusion.remove_lock()
        self.get_logger().info('已停车并移除跟随锁')


def _slew(prev, target, max_delta):
    """速度变化限幅，让动作平滑不抖动。"""
    if target > prev + max_delta:
        return prev + max_delta
    if target < prev - max_delta:
        return prev - max_delta
    return target


def main(args=None):
    rclpy.init(args=args)
    node = LidarFollowNode()

    # ------------------------------------------------------------------
    # 信号处理：确保被 launch / timeout / Ctrl+C 杀掉时都能安全停车
    # ------------------------------------------------------------------
    # 踩过的坑：只用 try/finally + KeyboardInterrupt 是不够的。
    # `timeout` 发的是 SIGTERM，`ros2 launch` 退出时转发的是 SIGINT，
    # 但两者都不会让 rclpy.spin() 抛 KeyboardInterrupt ——
    # 结果 finally 里的「发零速度 + 删锁文件」根本不会执行，
    # 留下孤儿进程和残留锁（残留锁会让导航永远无法启动）。
    #
    # 这里显式注册 handler：收到信号先安全停车，再让 spin 退出。
    _shutdown_done = {'flag': False}

    def _safe_shutdown(*_):
        if _shutdown_done['flag']:
            return
        _shutdown_done['flag'] = True
        try:
            node.shutdown()
        except Exception:                           # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, _safe_shutdown)
        except Exception:                           # noqa: BLE001
            pass

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _safe_shutdown()
        try:
            node.destroy_node()
        except Exception:                           # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
