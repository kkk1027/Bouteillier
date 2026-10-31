#!/usr/bin/env python3
"""
lidar_follow.py
单线激光雷达目标跟随节点。

================================================================
控制目标
================================================================

让小车与「前方最近障碍物」保持约 50cm，并对准它。

    ±75° 搜索扇区（未锁定时）
        ╲    │    ╱
         ╲   │   ╱
          ╲  │  ╱
        ┌───────┐
        │  车   │  ← 雷达在车体正中心，装得很低
        └───────┘

处理流程（每次扫描，5Hz）：

    LaserScan
       │
       ├─ 1. 全向提取有效点（几何过滤：NaN/Inf/量程）
       │
       ├─ 2. 聚类
       │      全向簇 → 前后方安全检查
       │      扇区内簇 → 目标选择
       │
       ├─ 3. 目标选择（两路并行，取更近的那个）
       │      a) 双腿对：两个小簇 + 两点中点  ← 人腿工况专用
       │      b) 单簇：点数够多的实体目标（纸箱、椅子、侧身站的人）
       │      c) 首次锁定要连续确认 3 帧，滤掉一闪而过的噪声
       │
       ├─ 4. 卡尔曼滤波 (d, θ) 各一个 CV 模型
       │
       └─ 5. 三态状态机 + PD 控制 → /cmd_vel

================================================================
为什么需要「双腿对」
================================================================

雷达装在车体下沿（离地约 10cm），扫不到人的躯干，只有两条小腿。
两条腿在极坐标里是**互不相连的两个小簇**，每条腿通常只有 1~4 个点：

        (0.63m,+10°) ●        ● (0.63m,-10°)
                      ╲      ╱
                       ╲    ╱
                      ┌──────┐
                      │  车  │
                      └──────┘

旧的单簇逻辑要求一个簇至少 5 个点 → 两条腿全被当噪声丢掉 → 永远锁不上。
就算偶尔某条腿凑够点数被锁定，目标也会在两条腿之间来回跳，
角度瞬抖 ±10°，车跟着抽。

所以：找「相邻的两个小簇」，取两点**中点**当目标。
中点≈人体中心线，既不左右跳，也正是「保持 50cm」该量到的位置。
锁定之后交给卡尔曼持续预测，单帧看不到某条腿也能顶住。

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

# 调试话题的布局
DEBUG_FIELDS = ('dist', 'angle', 'dist_rate', 'angle_rate',
                'n_clusters', 'state', 'lock_hits', 'kind')

# kind 字段的编码：0=未锁定 1=单簇 2=双腿对 3=单腿兜底
KIND_CODES = {'': 0, 'single': 1, 'pair': 2, 'leg': 3}


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

    # ---- 双腿成对锁定 ----
    # 雷达装在车体下沿，扫不到人躯干，只有两条小腿。
    # 每条腿在极坐标里是独立小簇（1~4 点），单簇门槛 min_points=5
    # 会把它们全部当噪声丢掉 —— 这就是「能跟但锁不上」的根因。
    # 这里改成：找「相邻的两个小簇」，取两点中点当目标。
    'pair_enable': True,
    # 成对的两个点各自的距离上限（用户要求：不超过 1m）
    'pair_lock_max_range': 1.00,
    'pair_track_max_range': 1.20,
    # 两点的直线距离必须落在 [pair_min_sep, pair_max_sep] 内
    'pair_max_sep': 0.60,
    # 太小说明是同一个物体被切成两簇，不必当中点处理
    'pair_min_sep': 0.05,
    # 两条腿基本在同一深度；一近一远的是「腿 + 墙」，不是一双腿
    # 0.22m ≈ 一步的步幅，人走路时前后腿的最大深度差也超不过它
    'pair_max_dist_diff': 0.22,
    # 两个簇点数之和的下限。人腿很细，1+1=2 也要认
    'pair_min_points': 2,
    # 人腿角度很窄（几度），墙/柜子几十度 —— 用它把墙排除
    'pair_max_span_deg': 45.0,
    # 两点的**横向**间距（垂直于视线）上限。横向就是「腿距」，
    # 真人约 0.22m，极少超过 0.5m。
    # ★ 专治「帧间随机跳动的噪声点凑成一对」：
    #   实机仿真曾出现间距 0.56m、角度差 49° 的假双腿，
    #   只靠直线间距 60cm 拦不住，横向分量能拦住。
    'pair_max_lateral_sep': 0.50,
    # 两点的**纵向**错位上限（沿视线）。
    # 并排站时两腿几乎前后对齐；侧身站时也会前后重叠。
    # 真正一前一后差 30cm 以上的，只能是两个不相关的物体。
    'pair_max_along_sep': 0.25,

    # ---- 单腿兜底 ----
    # 人走路时两条腿会互相遮挡，经常只有一条腿可见。
    # 这时双腿对不成立，但目标不能丢 —— 用单腿的**距离**继续跟，
    # 角度交给卡尔曼预测。因为单腿偏离身体中心 ±10° 左右，
    # 直接用它反角度会让车左右抽。
    'leg_fallback_enable': True,
    # 只认小簇（点数 <= 此值）。5 点以上属于实体目标，走正常路径
    'leg_max_points': 4,
    # 与预测距离的容差
    'leg_max_dist_diff': 0.35,
    # 未锁定时的搜索扇区半角。150° 意味着站在车侧面的人也能被认出来
    'lock_sector_half_angle_deg': 75.0,
    # 已锁定时允许的角度跳变。距离近邻但角度差太多的不是同一目标
    'max_angle_jump_deg': 25.0,
    # ---- 锁定后的稳定观察期 ----
    # 刚锁定的这丝秒内只观察、不动作。
    #
    # 为什么需要：上面所有判据再严，也拦不住「极端巧合」。
    # 实测在完全随机的合成散点下（物理上不存在，只做极限测试）
    # 跑长了仍可能偶尔误锁。有了这段观察期，即使误锁也动不了车：
    # 噪声在几帧内就会跳走，触发丢失或跳变保护。
    #
    # 正常使用无感知：人站好 → 锁定 → 本就应该等人开始走才动。
    'lock_settle_time': 1.0,
    # 首次锁定要连续确认的帧数（雷达 5Hz，5 帧 = 1.0s）
    # 噪声点是一闪而过的，人站在那里是持续的 —— 用时间换可靠性。
    #
    # ★ 5 帧不是拍脑袋定的：纯逻辑层反复跑了 800 次统计（每帧 3 个
    #   完全随机的散点，模拟最坏情况）：
    #     帧数  tol=0.15m/8°   tol=0.10m/6°
    #      3         9.1%          2.5%
    #      4         0.6%          0.1%
    #      5         0.0%          0.0%
    #   而真人双腿（腿距 0.22m±2cm、距离 0.62m±3cm）在以上所有
    #   配置下都是 100% 锁定成功。所以取 5 帧 + 0.10m/6°。
    #   代价只是站好之后多等 1 秒。
    'lock_confirm_frames': 5,
    # 确认期间允许多少帧「整帧看不到目标」（可能被挡住了）
    'lock_confirm_miss': 2,
    # 确认期间中点位置的容差。
    # 0.10m/帧 在 5Hz 下相当于 0.5m/s，正常步速；
    # 再放宽（比如 0.25m）就等于允许 1.25m/s，随机噪声很容易凑上。
    'lock_confirm_tol_dist': 0.10,
    'lock_confirm_tol_angle_deg': 6.0,
    # 确认期间还要求「两腿间距」稳定。
    # ★ 真实人的腿距几乎不变（不随距离变），噪声两点却每帧都变。
    #   这条比中点容差更能拦住「随机点凑巧凑成一对」。
    'lock_confirm_tol_sep': 0.20,
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

    # ---- 目标运动合理性 ----
    # 卡尔曼估出的径向速度超过这个值 → 目标不可信，停车。
    #
    # 这是拦「帧间随机跳动的杂散点」的最后一道阀。
    # 几何判据（同深度、腿距、横向宽度）堆到极限也拦不住所有巧合，
    # 但噪声有一个绕不过去的特征：位置突变 → 速度估计爆表。
    # 真人快走也就 1.0m/s，再算上本车 0.22m/s 的相对速度，
    # 1.5m/s 已经留足余量。
    'max_target_speed': 1.50,

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
        self.last_clusters = []      # 搜索扇区内的簇，给目标选择
        self.guard_clusters = []     # 窄前方扇区，给 32cm 刹车用
        self.all_clusters = []       # 全向簇，给安全检查

        # 首次锁定确认计数（防噪声点一闪就锁）
        self.lock_hits = 0
        self.lock_miss = 0
        self.lock_cand = None        # (dist, angle) 待确认候选
        self.lock_kind = ''          # 'single' / 'pair' / 'leg'
        self.lock_time = 0.0         # 锁定时刻（用于稳定观察期）

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

        self.pair_enable = bool(g('pair_enable'))
        self.pair_lock_max_range = float(g('pair_lock_max_range'))
        self.pair_track_max_range = float(g('pair_track_max_range'))
        self.pair_max_sep = float(g('pair_max_sep'))
        self.pair_min_sep = float(g('pair_min_sep'))
        self.pair_max_dist_diff = float(g('pair_max_dist_diff'))
        self.pair_min_points = int(g('pair_min_points'))
        self.pair_max_span_deg = float(g('pair_max_span_deg'))
        self.pair_max_lateral_sep = float(g('pair_max_lateral_sep'))
        self.pair_max_along_sep = float(g('pair_max_along_sep'))
        self.leg_fallback_enable = bool(g('leg_fallback_enable'))
        self.leg_max_points = int(g('leg_max_points'))
        self.leg_max_dist_diff = float(g('leg_max_dist_diff'))
        self.lock_sector_rad = math.radians(
            float(g('lock_sector_half_angle_deg')))
        self.max_angle_jump = math.radians(float(g('max_angle_jump_deg')))
        self.settle_time = float(g('lock_settle_time'))

        self.lock_confirm_frames = int(g('lock_confirm_frames'))
        self.lock_confirm_miss = int(g('lock_confirm_miss'))
        self.lock_tol_dist = float(g('lock_confirm_tol_dist'))
        self.lock_tol_angle = math.radians(
            float(g('lock_confirm_tol_angle_deg')))
        self.lock_tol_sep = float(g('lock_confirm_tol_sep'))

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
        self.max_target_speed = float(g('max_target_speed'))

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

        # ---- 双腿配对参数自检 ----
        if self.pair_enable:
            if self.pair_min_sep >= self.pair_max_sep:
                self.get_logger().error(
                    'pair_min_sep(%.2f) >= pair_max_sep(%.2f)，'
                    '配对永远不成立，双腿锁定会完全失效'
                    % (self.pair_min_sep, self.pair_max_sep))
            if self.pair_min_points < 2:
                self.get_logger().warn(
                    'pair_min_points(%d) < 2，单个噪声点就能成为候选'
                    % self.pair_min_points)
            if self.pair_lock_max_range > self.pair_track_max_range:
                self.get_logger().warn(
                    'pair_lock_max_range(%.2f) > pair_track_max_range(%.2f)，'
                    '锁定后反而看不远，会立刻丢失'
                    % (self.pair_lock_max_range, self.pair_track_max_range))
            if self.lock_sector_rad < self.sector_rad:
                self.get_logger().warn(
                    'lock_sector_half_angle_deg(%.0f°) < '
                    'sector_half_angle_deg(%.0f°)，锁定扇区比安全扇区还窄'
                    % (math.degrees(self.lock_sector_rad),
                       math.degrees(self.sector_rad)))

        if self.lock_confirm_frames < 1:
            self.get_logger().warn(
                'lock_confirm_frames(%d) < 1，首次锁定不做时间确认，'
                '容易被单帧噪声骗到' % self.lock_confirm_frames)

    def _log_startup(self):
        self.get_logger().info('=' * 58)
        self.get_logger().info('雷达跟随节点已启动')
        self.get_logger().info('  扫描话题 : %s' % self.scan_topic)
        self.get_logger().info('  输出话题 : %s' % self.cmd_topic)
        self.get_logger().info('  锁定范围 : 单簇 %.2fm / 双腿对 %.2fm / 跟踪 %.2fm'
                               % (self.lock_max_range,
                                  self.pair_lock_max_range,
                                  self.track_max_range))
        if self.pair_enable:
            self.get_logger().info(
                '  双腿配对 : 两点间距 %.2f~%.2fm，深度差 <=%.2fm，'
                '点数之和 >=%d，%d 帧确认'
                % (self.pair_min_sep, self.pair_max_sep,
                   self.pair_max_dist_diff, self.pair_min_points,
                   self.lock_confirm_frames))
        self.get_logger().info(
            '  距离死区 : 后退<%.2f  停 %.2f~%.2f  前进>%.2f'
            % (self.retreat_enter, self.approach_exit,
               self.approach_enter, self.approach_enter))
        self.get_logger().info(
            '  扇区     : 搜索 ±%.0f° / 安全 ±%.0f°  (后方 ±%.0f°)'
            % (math.degrees(self.lock_sector_rad),
               math.degrees(self.sector_rad), math.degrees(self.rear_rad)))
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

        # 搜索扇区：未锁定时用宽扇区（能把站在侧面的人框进来），
        # 已锁定时至少覆盖目标当前方位，避免目标偏到一边就看不见。
        search_rad = self._search_half_angle()
        search_pts = geometry.sector_filter(all_pts, search_rad)
        self.last_clusters = geometry.cluster_points(
            search_pts, self.cluster_angle_gap, self.cluster_range_gap)

        # 窄前方扇区：只给「前方 32cm 有障碍就刹车」用。
        # 单独算一份，这样搜索扇区变宽不会悄悄改变安全语义。
        guard_pts = geometry.sector_filter(all_pts, self.sector_rad)
        self.guard_clusters = geometry.cluster_points(
            guard_pts, self.cluster_angle_gap, self.cluster_range_gap)

        ref_d = self.target_dist if (self.locked and self.target_valid) else 0.0
        ref_a = self.target_angle if (self.locked and self.target_valid) else 0.0

        pick = geometry.select_follow_target(
            self.last_clusters, self.locked, ref_d, ref_a,
            self.lock_max_range, self.track_max_range,
            min_points=self.min_cluster_points,
            max_jump=self.max_target_jump,
            max_angle_jump=self.max_angle_jump,
            pair_enable=self.pair_enable,
            pair_lock_max_range=self.pair_lock_max_range,
            pair_track_max_range=self.pair_track_max_range,
            pair_max_sep=self.pair_max_sep,
            pair_min_sep=self.pair_min_sep,
            pair_max_dist_diff=self.pair_max_dist_diff,
            pair_min_points=self.pair_min_points,
            pair_max_span_deg=self.pair_max_span_deg,
            pair_max_lateral_sep=self.pair_max_lateral_sep,
            pair_max_along_sep=self.pair_max_along_sep,
            leg_fallback=self.leg_fallback_enable,
            leg_max_points=self.leg_max_points,
            leg_max_dist_diff=self.leg_max_dist_diff)

        if pick is None:
            self._handle_no_target(stamp)
            return

        dist = pick['dist']
        angle = pick['angle']

        # 首次锁定判定
        if not self.locked:
            if not self._confirm_lock(dist, angle, pick.get('sep', 0.0),
                                      pick['kind']):
                self.target_valid = False
                return
            self.locked = True
            self.lock_kind = pick['kind']
            self.lock_time = stamp
            self.stat_locked += 1
            self.tracker.reset()
            if pick['kind'] == 'pair':
                la, lb = pick['leg_a'], pick['leg_b']
                self.get_logger().info(
                    '🔒 已锁定【双腿对】：%.2fm/%+.1f° + %.2fm/%+.1f°  '
                    '→ 中点 %.2fm/%+.1f°，间距 %.2fm(横%.2f)（%d 点），开始跟随'
                    % (la[0], math.degrees(la[1]),
                       lb[0], math.degrees(lb[1]),
                       dist, math.degrees(angle),
                       pick['sep'], pick.get('lateral', 0.0), pick['n']))
            else:
                self.get_logger().info(
                    '🔒 已锁定【单簇目标】：%.2fm / %+.1f°（%d 点），开始跟随'
                    % (dist, math.degrees(angle), pick['n']))

        # 单腿兜底：只有一条腿看得见 → 只用距离，角度走预测
        if pick.get('angle_valid', True):
            fd, fa, dv, av = self.tracker.observe(dist, angle, stamp)
        else:
            fd, fa, dv, av = self.tracker.observe_dist_only(dist, stamp)

        self.target_dist = fd
        self.target_angle = fa
        self.target_dist_rate = dv
        self.target_angle_rate = av
        self.target_valid = True

    # --------------------------------------------------------
    # 首次锁定：时间确认
    # --------------------------------------------------------
    def _search_half_angle(self):
        """本帧用多大的搜索扇区。"""
        if not (self.locked and self.target_valid):
            return min(self.lock_sector_rad, math.pi)
        # 目标偏到 ±60° 时，±30° 的扇区就看不见它了 → 跟着目标放宽
        need = abs(self.target_angle) + math.radians(20.0)
        return min(max(self.sector_rad, need), math.pi)

    def _confirm_lock(self, dist, angle, sep=0.0, kind=''):
        """首次锁定要连续多帧看到「同一个位置、同一个形状」的东西才承认。

        原因：单个 1~2 点的噪声簇距离可能是 0.8m，一闪而过。
        人站在那里是持续的，噪声不是。用时间换可靠性比单纯调
        点数门槛靠谱得多。

        除了中点位置，还要求两腿间距稳定：真人腿距是固定的，
        随机噪声两点之间的距离每帧都不同。

        返回 True 表示可以锁定。
        """
        if self.lock_confirm_frames <= 1:
            return True

        if self.lock_cand is None:
            self.lock_cand = (dist, angle, sep, kind)
            self.lock_hits = 1
            self.lock_miss = 0
            return False

        cd, ca, cs, ck = self.lock_cand
        same = (abs(dist - cd) <= self.lock_tol_dist
                and abs(geometry.wrap_angle(angle - ca)) <= self.lock_tol_angle)

        # 两腿对：间距也必须稳定，否则不是同一个人
        if same and kind == 'pair' and ck == 'pair':
            if abs(sep - cs) > self.lock_tol_sep:
                same = False

        if not same:
            # 看到了目标，但位置或形状变了 → 这不是刚才那个东西，
            # 从这一帧重新开始计数，**不做任何容错**。
            #
            # 为什么不容错：噪声点每帧位置都在跳。只要允许
            # 「命中-漏检-命中」把命中数累加，跳动的噪声也能
            # 凑满 3 帧 —— 实测会锁到一对「间距 0.35m、横向 0.35m」
            # 的假双腿，然后以 0.22m/s 真的前进。
            #
            # 真正需要的容错是「整帧看不到目标」（可能被挡住了），
            # 那个在 _handle_no_target 里由 lock_confirm_miss 处理。
            # 这两种情况必须分开对待，否则就会把噪声也容错进来。
            self.lock_cand = (dist, angle, sep, kind)
            self.lock_hits = 1
            self.lock_miss = 0
            return False

        self.lock_hits += 1
        self.lock_miss = 0
        self.lock_cand = (dist, angle, sep, kind)      # 跟着目标微动

        if self.lock_hits >= self.lock_confirm_frames:
            self.lock_cand = None
            self.lock_miss = 0
            return True
        return False

    def _handle_no_target(self, stamp):
        if not self.locked:
            self.target_valid = False
            self.lock_miss += 1
            if self.lock_miss > self.lock_confirm_miss:
                self.lock_cand = None
                self.lock_hits = 0
                self.lock_miss = 0
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

        # ---- 安全 3：刚锁定的稳定观察期 ----
        # 先观察再动。即使锁上的是噪声，这几帧内它也会跳走。
        if self.settle_time > 0.0:
            left = self.settle_time - (now - self.lock_time)
            if left > 0.0:
                self.state = ST_HOLD
                return 0.0, 0.0, '锁定稳定观察中（还剩 %.1fs）' % left

        # ---- 安全 4：目标运动不合常理 ----
        # 杂散点会让距离估计剧烈震荡，速度估计尧到 1.5m/s 以上。
        # 保持锁定状态（等它自己稳定下来），但不允许移动。
        if abs(self.target_dist_rate) > self.max_target_speed:
            return 0.0, 0.0, ('目标速度 %.1fm/s 超过上限 %.1fm/s，停车'
                              % (self.target_dist_rate,
                                 self.max_target_speed))

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

        # ---- 安全 5：后方有障碍不许后退 ----
        if lin < 0.0 and not geometry.rear_is_clear(
                self.all_clusters, self.rear_rad, self.rear_range):
            lin = 0.0
            reason += ' | 后方%.1fm内有障碍，不后退' % self.rear_range

        # ---- 安全 6：前方有比目标更近的东西 ----
        if lin > 0.0 and d > self.front_guard:
            nearest = geometry.front_nearest(self.guard_clusters)
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
                float(self.lock_hits if not self.locked else 0),
                float(KIND_CODES.get(self.lock_kind, 0)),
            ]
            self.debug_pub.publish(m)

    def publish_state(self, reason):
        s = String()
        s.data = '%s|%.2f|%.1f|%s|%s|%d' % (
            self.state, self.target_dist, math.degrees(self.target_angle),
            'LOCK' if self.locked else 'NOLOCK',
            self.lock_kind or '-', self.lock_hits)
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
