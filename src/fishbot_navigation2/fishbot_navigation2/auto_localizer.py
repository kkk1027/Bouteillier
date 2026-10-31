#!/usr/bin/env python3
"""导航启动后自动完成 AMCL 全局定位，无需人工给初始位姿。

背景
----
`nav2_amcl` 有两种起手方式：

1. `set_initial_pose: true` + `initial_pose.x/y/yaw`
   —— 启动即把机器人当成参数里写死的那个位姿（通常是地图原点）。
   如果小车实际不在那里，AMCL 会从一个错误的先验开始，只能靠
   recovery 随机粒子慢慢挣扎，表现为"定位漂了/要等很久"。

2. `set_initial_pose: false` + 调用 `/reinitialize_global_localization`
   —— AMCL 把粒子云**均匀撒满整张地图的可行区域**，之后每帧激光
   都会淘汰掉与地图不符的粒子，随小车移动自动收敛到真实位姿。
   这就是本节点实现的"自动收敛"，不需要人工在 RViz 点 2D Pose Estimate。

参考实现
--------
`/home/user/wheeltec_ros2/src/wheeltec_robot_nav2`
  - `param/wheeltec_params/param_four_wheel_diff.yaml`：`set_initial_pose: False`，
    `max_particles: 2000 / min_particles: 500`
  - `launch/wheeltec_nav2.launch.py`：`nomotion_update_period_sec: 0.5`
    —— 静止时也周期性触发 AMCL 更新，否则车不动粒子云就不会被激光淘汰

本节点做的事
------------
1. 等 `/map`（latched）与 `/reinitialize_global_localization` 服务就绪；
2. 调用该服务，让粒子云全局散布；
3. 周期性调用 `/request_nomotion_update`，使小车静止时 AMCL 也能用激光更新
   （否则必须先手动推车才会收敛）；
4. 监控 `/amcl_pose` 的协方差，连续稳定低于阈值即判定收敛并记录日志；
5. 若期间收到外部 `/initialpose`（RViz 点选、巡逻模块、网页端），
   立即让位，不再干预 —— 尊重人工/其他模块的设置。

安全边界
--------
本节点**只调用 AMCL 服务、只发布 /initialpose**，
不发布任何 /cmd_vel，不会让小车移动。
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

# PoseWithCovarianceStamped 6x6 协方差矩阵按行优先展开
COV_X = 0
COV_Y = 7
COV_YAW = 35


class AutoLocalizer(Node):
    """AMCL 自动全局定位状态机。"""

    STATE_WAITING_MAP = 'waiting_map'
    STATE_TRIGGERING = 'triggering'
    STATE_CONVERGING = 'converging'
    STATE_CONVERGED = 'converged'
    STATE_DEFERRED = 'deferred_to_external'
    STATE_TIMEOUT = 'timeout'

    def __init__(self) -> None:
        super().__init__('auto_localizer')

        # ---------------- 参数 ----------------
        self.declare_parameter('auto_global_localization', True)
        # 若为 true，则不发全局散布，而是用下面的粗先验初始化
        self.declare_parameter('use_initial_pose_prior', False)
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)
        # 粗先验的协方差（越大越不信任这个先验）；默认 0.5m / 0.26rad
        self.declare_parameter('initial_pose_cov_xy', 0.25)
        self.declare_parameter('initial_pose_cov_yaw', 0.0685)

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        # 激光话题：没有激光 AMCL 无法匹配，收敛永远不可能发生
        # （实测踩过：桥接板不收发字节 → /scan 消失 → 粒子云永不收敛）
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_stale_sec', 3.0)
        self.declare_parameter(
            'global_localization_service', '/reinitialize_global_localization')
        self.declare_parameter(
            'nomotion_update_service', '/request_nomotion_update')

        self.declare_parameter('startup_delay_sec', 3.0)
        # 静止更新周期。
        #
        # 2026-10-31 二次实测后从 0.25 回调到 1.0：
        # 0.25s(4Hz) 会让 AMCL 对**同一帧**激光反复重采样。小车静止时
        # 运动模型不产生任何噪声，重采样出的副本彼此完全重合，粒子方差
        # 被单调压到 0（实测 σ_xy 从 0.0156 塔到 0.0001m）。
        # 塔缩后滤波器失去全部备选假设，recovery 注入的随机粒子一旦
        # 权重更高，位姿就会瞬间跳到别处（实测跳了 1.0m）。
        # 所以：宁可慢一点，也不能高频重复喂同一帧。
        self.declare_parameter('nomotion_update_period_sec', 1.0)
        self.declare_parameter('tick_period_sec', 0.5)
        # 收敛判据：sqrt(方差) 低于阈值
        self.declare_parameter('converge_cov_xy', 0.08)
        self.declare_parameter('converge_cov_yaw', 0.05)
        self.declare_parameter('converge_hold_sec', 3.0)
        self.declare_parameter('timeout_sec', 120.0)
        self.declare_parameter('timeout_retries', 2)
        # 收敛后**停止**请求 nomotion。
        # 之前设成 False（收敛后仍以 2s 周期刷新）是为了让 map→odom
        # 跟上 odom 漂移。但固件加编码器死区后，静止时 odom 已经不漂
        # （实测 2252 帧 wz 全为 0），继续刷新只会不断重采样导致粒子塔缩。
        self.declare_parameter('stop_nomotion_after_converge', True)
        self.declare_parameter('nomotion_update_period_after_converge_sec', 2.0)
        # 粒子塔缩判据：σ 低于物理下限即认为滤波器已退化。
        # 0.05m 栅格 + 最少 500 个粒子不可能达到 5mm 精度。
        self.declare_parameter('collapse_cov_xy', 0.005)
        self.declare_parameter('collapse_cov_yaw', 0.01)
        self.declare_parameter('max_collapse_recoveries', 3)
        self.declare_parameter('defer_on_external_initial_pose', True)
        self.declare_parameter('log_period_sec', 10.0)

        self.auto_global = bool(
            self.get_parameter('auto_global_localization').value)
        self.use_prior = bool(
            self.get_parameter('use_initial_pose_prior').value)
        self.initial_x = float(self.get_parameter('initial_pose_x').value)
        self.initial_y = float(self.get_parameter('initial_pose_y').value)
        self.initial_yaw = float(self.get_parameter('initial_pose_yaw').value)
        self.initial_cov_xy = float(
            self.get_parameter('initial_pose_cov_xy').value)
        self.initial_cov_yaw = float(
            self.get_parameter('initial_pose_cov_yaw').value)

        self.startup_delay_sec = float(
            self.get_parameter('startup_delay_sec').value)
        self.nomotion_period = max(
            0.1, float(self.get_parameter('nomotion_update_period_sec').value))
        self.tick_period = max(
            0.1, float(self.get_parameter('tick_period_sec').value))
        self.converge_cov_xy = float(
            self.get_parameter('converge_cov_xy').value)
        self.converge_cov_yaw = float(
            self.get_parameter('converge_cov_yaw').value)
        self.converge_hold_sec = float(
            self.get_parameter('converge_hold_sec').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.stop_nomotion_after_converge = bool(
            self.get_parameter('stop_nomotion_after_converge').value)
        self.nomotion_period_after_converge = max(
            0.2, float(self.get_parameter(
                'nomotion_update_period_after_converge_sec').value))
        self.timeout_retries = int(
            self.get_parameter('timeout_retries').value)
        self.collapse_cov_xy = float(
            self.get_parameter('collapse_cov_xy').value)
        self.collapse_cov_yaw = float(
            self.get_parameter('collapse_cov_yaw').value)
        self.max_collapse_recoveries = int(
            self.get_parameter('max_collapse_recoveries').value)
        self.defer_on_external = bool(
            self.get_parameter('defer_on_external_initial_pose').value)
        self.log_period = float(self.get_parameter('log_period_sec').value)
        self.scan_stale_sec = float(
            self.get_parameter('scan_stale_sec').value)

        # ---------------- 状态 ----------------
        self.state = self.STATE_WAITING_MAP
        self._map_ready = False
        self._map_info = ''
        self._start_time = self.get_clock().now()
        self._trigger_sent = False
        self._prior_sent = False
        # 自身发布 /initialpose 的时间与坐标（用于过滤自环回显）
        self._self_prior_stamp = None
        self._self_prior_xy = (0.0, 0.0)
        self._converged_since = None
        self._last_nomotion_time = self._start_time
        self._last_log_time = self._start_time
        self._external_initial_pose = 0
        self._latest_cov = None
        self._latest_pose = None
        # 地图数据（供假收敛校验）
        self._map_data = None
        self._map_dims = None
        # 假收敛计数：连续几次收敛判定都无法通过地图校验
        self._fake_converge_warned = False
        self._nomotion_requests = 0
        # 粒子塔缩恢复次数 / 超时重试次数
        self._collapse_recoveries = 0
        self._timeout_retries_used = 0
        self._nomotion_failures = 0
        self._finished = False
        # 激光健康度：用于区分「车没动」与「根本没激光」
        self._scan_count = 0
        self._last_scan_time = None
        self._warned_no_scan = False

        # ---------------- 订阅 ----------------
        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value),
            self._map_callback, map_qos)
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('amcl_pose_topic').value),
            self._amcl_pose_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('initial_pose_topic').value),
            self._initial_pose_callback, 10)
        # 激光用 BEST_EFFORT：与 RELIABLE 发布者（ydlidar）兼容，
        # 反过来 RELIABLE 订阅者会与 BEST_EFFORT 发布者不兼容。
        scan_qos = QoSProfile(depth=5,
                              reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self._scan_callback, scan_qos)

        # ---------------- 发布/服务 ----------------
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter('initial_pose_topic').value), 10)
        self._global_loc_client = self.create_client(
            Empty, str(self.get_parameter('global_localization_service').value))
        self._nomotion_client = self.create_client(
            Empty, str(self.get_parameter('nomotion_update_service').value))

        # ---------------- 主循环 ----------------
        self._timer = self.create_timer(self.tick_period, self._tick)

        if not self.auto_global:
            if self.use_prior:
                self.get_logger().info(
                    '自动全局定位已关闭，改为在 %.1fs 后发布粗先验 '
                    '(x=%.2f, y=%.2f, yaw=%.2f, cov_xy=%.2f)'
                    % (self.startup_delay_sec, self.initial_x, self.initial_y,
                       self.initial_yaw, self.initial_cov_xy))
            else:
                self.get_logger().warn(
                    'auto_global_localization=false 且 use_initial_pose_prior=false：'
                    '本节点不做任何事，AMCL 需要外部提供 /initialpose')
                self.state = self.STATE_DEFERRED
                self._finished = True
        else:
            self.get_logger().info(
                '自动全局定位已启用：等待 /map 与 AMCL 服务就绪后，'
                '调用 reinitialize_global_localization 让粒子云全局散布，'
                '随小车移动自动收敛（无需人工给初始位姿）')

    # ============================================================
    # 回调
    # ============================================================
    def _map_callback(self, msg: OccupancyGrid) -> None:
        if self._map_ready:
            return
        self._map_ready = True
        width = int(msg.info.width)
        height = int(msg.info.height)
        res = float(msg.info.resolution)
        origin = msg.info.origin.position
        self._map_info = (
            '%dx%d @ %.3f m/px, 覆盖 %.1fm x %.1fm, 原点(%.2f, %.2f)'
            % (width, height, res, width * res, height * res,
               origin.x, origin.y))
        self.get_logger().info('收到地图：%s' % self._map_info)

        # 保存地图，供收敛时做「假收敛」校验（见 _check_pose_on_map）
        self._map_data = msg.data
        self._map_dims = (width, height, res, origin.x, origin.y)

        free = sum(1 for v in msg.data if 0 <= v < 65)
        occ = sum(1 for v in msg.data if v >= 65)
        unknown = sum(1 for v in msg.data if v < 0)
        total = max(1, width * height)
        free_pct = free * 100.0 / total
        occ_pct = occ * 100.0 / total
        unk_pct = unknown * 100.0 / total
        self.get_logger().info(
            '地图构成：空闲 %.1f%%  占用(墙壁) %.1f%%  未知 %.1f%%'
            % (free_pct, occ_pct, unk_pct))

        # ==========================================================
        # 地图质量判据：看**障碍物占比**，不是空闲占比
        # ----------------------------------------------------------
        # 为什么：AMCL 的似然场只在靠近障碍物处有梯度。空闲区再大
        # 也不会提供区分度——粒子飘在空地上，激光一路射到远处墙，
        # 得分都差不多。所以决定收敛质量的是「墙壁占多少」。
        #
        # 本项目实测踩坑（已修）：map_saver 用灰度 205 表示未知像素，
        # 其占用概率是 (255-205)/255 = 0.1961。若地图 yaml 的
        # free_thresh 设成 0.25，map_server 的 trinary 模式会把这些
        # 未知像素全判成**空闲**，于是 AMCL 看到的是「93% 空闲 +
        # 6.8% 墙壁」。墙太少 → 似然场几乎处处平坦 → 静止收敛极慢
        # 且容易假收敛。现已在 manual_mapping_manager 和地图 yaml 里
        # 把 free_thresh 统一为 0.196（ROS 官方默认值）。
        #
        # 经验阈值：墙壁占比 < 6% 时收敛质量会明显变差。
        # ==========================================================
        if occ_pct < 6.0:
            self.get_logger().warn(
                '⚠️ 地图墙壁仅占 %.1f%%，似然场区分度不足。\n'
                '   空闲区虽然占 %.1f%%，但空地不提供定位信息——'
                '粒子飘在空地上时激光都射到远处墙，得分相近、无法淘汰。\n'
                '   后果：静止收敛慢（实测约 78 秒），且容易「假收敛」'
                '（σ 很小但位置错）。\n'
                '   建议：重建一张覆盖完整、墙壁轮廓清晰的图；'
                '当前可靠缓慢原地转动辅助收敛。'
                % (occ_pct, free_pct))
        if unk_pct > 40.0:
            self.get_logger().warn(
                '⚠️ 地图未知区占 %.1f%%，说明建图时有大片区域没扫到。'
                '建议重建地图' % unk_pct)

    def _scan_callback(self, msg: LaserScan) -> None:
        self._scan_count += 1
        self._last_scan_time = self.get_clock().now()
        if self._scan_count == 1:
            self.get_logger().info(
                '收到激光 %s：%d 束，量程 %.2f~%.2fm'
                % (self.get_parameter('scan_topic').value,
                   len(msg.ranges), msg.range_min, msg.range_max))

    def _scan_age(self) -> float:
        """距上一帧激光的秒数；从未收到则返回 inf。"""
        if self._last_scan_time is None:
            return float('inf')
        return (self.get_clock().now() -
                self._last_scan_time).nanoseconds / 1e9

    def _scan_ok(self) -> bool:
        return self._scan_age() <= self.scan_stale_sec

    def _scan_status_text(self) -> str:
        if self._scan_count == 0:
            return '无激光'
        age = self._scan_age()
        if age > self.scan_stale_sec:
            return '激光已停 %.1fs' % age
        return '激光正常'

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        cov = msg.pose.covariance
        self._latest_cov = (float(cov[COV_X]), float(cov[COV_Y]),
                            float(cov[COV_YAW]))
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._latest_pose = (float(p.x), float(p.y), yaw)

    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """外部（RViz/巡逻模块/网页）设置了初始位姿：让位，不再干预。

        注意：本节点在「粗先验」模式或服务降级时也会自己发 /initialpose，
        那些消息会回环到这里。必须识别并忽略，否则会误判成外部介入而
        立即停止工作。
        """
        if self._finished:
            return
        if self._is_self_published(msg):
            return
        self._external_initial_pose += 1
        if not self.defer_on_external:
            return
        p = msg.pose.pose.position
        self.get_logger().info(
            '检测到外部 /initialpose (x=%.2f, y=%.2f)：'
            '停止自动全局定位，改由外部设定主导' % (p.x, p.y))
        self.state = self.STATE_DEFERRED
        self._finished = True
        self._timer.cancel()

    def _is_self_published(self, msg: PoseWithCovarianceStamped) -> bool:
        """判断这帧 /initialpose 是否由本节点自己发出（自环过滤）。"""
        if self._self_prior_stamp is None:
            return False
        now = self.get_clock().now()
        age = (now - self._self_prior_stamp).nanoseconds / 1e9
        if age > 2.0:          # 太久远了，不认为是自己的
            return False
        p = msg.pose.pose.position
        dx = abs(p.x - self._self_prior_xy[0])
        dy = abs(p.y - self._self_prior_xy[1])
        return dx < 1e-3 and dy < 1e-3

    # ============================================================
    # 主循环
    # ============================================================
    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds / 1e9

    def _tick(self) -> None:
        if self._finished:
            return

        now = self.get_clock().now()
        elapsed = self._elapsed()

        # ---------- 阶段 0：等启动延时 ----------
        if elapsed < self.startup_delay_sec:
            return

        # ---------- 阶段 1：等地图与 AMCL 服务 ----------
        if self.state == self.STATE_WAITING_MAP:
            self._check_scan_health()
            if not self._map_ready:
                self._log_periodically(
                    '仍在等待 %s（AMCL 没有地图无法定位）'
                    % self.get_parameter('map_topic').value)
                self._check_timeout(elapsed)
                return

            if self.use_prior:
                self._publish_prior()
                self.state = self.STATE_CONVERGING
                return

            if not self._global_loc_client.service_is_ready():
                self._log_periodically(
                    '等待服务 %s 就绪（AMCL 尚未启动完成）'
                    % self._global_loc_client.srv_name)
                self._check_timeout(elapsed)
                return

            self._trigger_global_localization()
            return

        # ---------- 阶段 2：持续触发 nomotion 更新并判定收敛 ----------
        if self.state == self.STATE_CONVERGING:
            self._check_scan_health()
            self._check_timeout(elapsed)
            if self._finished:
                return

            self._maybe_request_nomotion(now)
            self._evaluate_convergence(now, elapsed)
            if self._finished:      # 本轮已收敛，不再打"进行中"
                return
            self._log_periodically(
                '全局定位进行中（已 %.0fs，%s）：位姿=%s 协方差=%s'
                % (elapsed, self._scan_status_text(),
                   self._fmt_pose(), self._fmt_cov()))
            return

        # ---------- 阶段 3：已收敛，但仍周期性刷新 map→odom ----------
        # AMCL 只处理激光时才重发 map→odom。完全停掉请求的话，静止时
        # map→odom 会冻结，odom→base_link 的任何漂移都会直接显示成
        # 「定位在飘」。这里保持低频请求，把底层误差周期性吸收掉。
        if self.state == self.STATE_CONVERGED:
            self._maybe_request_nomotion(now)

    # ============================================================
    # 动作
    # ============================================================
    def _trigger_global_localization(self) -> None:
        # 重试路径（坍缩恢复 / 超时重试）可能在 AMCL 尚未就绪时进来，
        # 这里兜一层，避免 call_async 打空导致 future 永远不回来、
        # 状态卡在 TRIGGERING 再也不推进。
        if not self._global_loc_client.service_is_ready():
            self.get_logger().warn(
                '服务 %s 尚未就绪，暂不重新撒粒子（下一轮再试）'
                % self._global_loc_client.srv_name)
            self.state = self.STATE_CONVERGING
            return
        self.state = self.STATE_TRIGGERING
        self.get_logger().info(
            '调用 %s：让 AMCL 把粒子云撒满整张地图'
            % self._global_loc_client.srv_name)
        future = self._global_loc_client.call_async(Empty.Request())

        def _done(fut):
            try:
                fut.result()
            except Exception as exc:  # noqa: BLE001 - 服务异常不该拖垮节点
                self.get_logger().error(
                    '全局定位调用失败：%s；将降级为发布大协方差先验' % exc)
                self._publish_prior()
                self.state = self.STATE_CONVERGING
                return
            self._trigger_sent = True
            self.get_logger().info(
                '全局定位已触发。请缓慢移动/旋转小车（或原地转动），'
                'AMCL 会随激光匹配逐步收敛；静止不动时收敛会很慢')
            self.state = self.STATE_CONVERGING

        future.add_done_callback(_done)

    def _publish_prior(self) -> None:
        if self._prior_sent:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        yaw = self.initial_yaw
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[COV_X] = self.initial_cov_xy
        cov[COV_Y] = self.initial_cov_xy
        cov[COV_YAW] = self.initial_cov_yaw
        msg.pose.covariance = cov
        # 连发几次，避免 AMCL 尚未订阅完成丢掉第一帧
        for _ in range(3):
            self._initial_pose_pub.publish(msg)
        self._prior_sent = True
        # 记下“这是我自己发的”，供 _is_self_published() 过滤回环
        self._self_prior_stamp = self.get_clock().now()
        self._self_prior_xy = (self.initial_x, self.initial_y)
        self.get_logger().info(
            '已发布粗先验 (x=%.2f, y=%.2f, yaw=%.2f, cov_xy=%.2f, cov_yaw=%.3f)'
            % (self.initial_x, self.initial_y, self.initial_yaw,
               self.initial_cov_xy, self.initial_cov_yaw))

    def _maybe_request_nomotion(self, now) -> None:
        # ==========================================================
        # 收敛后是否继续请求 nomotion 更新
        # ----------------------------------------------------------
        # AMCL 只在**处理到激光帧时**才会重发 map→odom 这个 TF。
        # 小车静止时因为没有运动，AMCL 默认不会更新粒子，也就不重发 TF。
        #
        # 一旦停止请求 nomotion，map→odom 就冻结了。此时 odom→base_link
        # 只要有一点点漂移，都会原封不动地出现在 map→base_link 上——
        # 也就是在 RViz 里看到 base_link 相对 map 一直飘。
        # 本项目实际踩过：收敛后停止请求，静止时定位看起来一直在飘。
        #
        # 所以默认改成「收敛后仍以更慢的频率继续请求」，让 AMCL 持续用
        # 最新激光帧把 map→odom 拉回去。这样即使底层 odom 有小误差，
        # 也会被周期性吸收掉，而不是持续累积到视觉可见。
        # ==========================================================
        if self.state == self.STATE_CONVERGED:
            if self.stop_nomotion_after_converge:
                return
            period = self.nomotion_period_after_converge
        else:
            period = self.nomotion_period

        if self._latest_cov is None:
            # AMCL 还没输出位姿，此时请求无意义
            return
        dt = (now - self._last_nomotion_time).nanoseconds / 1e9
        if dt < period:
            return
        self._last_nomotion_time = now
        if not self._nomotion_client.service_is_ready():
            return
        future = self._nomotion_client.call_async(Empty.Request())
        self._nomotion_requests += 1

        def _done(fut):
            try:
                fut.result()
            except Exception:  # noqa: BLE001 - 单次失败可忽略
                self._nomotion_failures += 1

        future.add_done_callback(_done)

    # ============================================================
    # 收敛判定
    # ============================================================
    def _check_pose_on_map(self):
        """校验当前位姿是否落在合理位置，识别「假收敛」。

        ★ 为什么需要这一步：
          协方差 σ 只反映**粒子云的一致性（precision）**，不反映
          **准确性（accuracy）**。如果所有粒子都收敛到同一个错误位置，
          σ 同样会很小——这就是 AMCL 经典的「假收敛」。

          静止时特别容易发生：缺少新观测去证伪那个错误的共识。
          实测就踩到过：σ_xy 已经 0.049，但位置是错的，必须移动
          一段时间才纠正过来。

          这里用两个廉价但有效的判据：
            1. 位姿是否落在**已知区域**（未知区 = 地图没覆盖，不可信）
            2. 位姿是否落在**障碍物上**（物理上不可能）

        Returns:
            (是否可信, 说明字符串)
        """
        if self._map_data is None or self._map_dims is None:
            return True, '（无地图数据，跳过校验）'
        if self._latest_pose is None:
            return True, '（无位姿）'

        width, height, res, ox, oy = self._map_dims
        x, y, _ = self._latest_pose
        # 世界坐标 -> 栅格索引（PGM 行 0 在顶部，故 y 需要翻转）
        col = int((x - ox) / res)
        row = height - 1 - int((y - oy) / res)

        if not (0 <= col < width and 0 <= row < height):
            return False, '位姿在地图范围之外 (%.2f, %.2f)' % (x, y)

        val = self._map_data[row * width + col]
        if val < 0:
            return False, '位姿落在未知区域（地图没覆盖这里）'
        if val >= 65:
            return False, '位姿落在障碍物上（物理上不可能）'
        return True, '位姿位于已知空闲区域（栅格值 %d）'

    def _evaluate_convergence(self, now, elapsed: float) -> None:
        if self._latest_cov is None:
            return
        var_x, var_y, var_yaw = self._latest_cov
        std_xy = math.sqrt(max(0.0, max(var_x, var_y)))
        std_yaw = math.sqrt(max(0.0, var_yaw))

        # ==========================================================
        # 粒子塔缩检测（particle deprivation）
        # ----------------------------------------------------------
        # 小车静止时反复 request_nomotion_update，AMCL 会对**同一帧**
        # 激光反复重采样。没有运动 → 运动模型不加噪声 → 重采样出的
        # 副本彼此完全重合 → 方差单调收缩到 0。
        #
        # 实测证据：σ_xy 从 0.0156m 塔到 0.0001m（!），随后暴涨到
        # 0.1428m，位姿瞬移 1.0m。因为塔缩后粒子云退化成单一副本，
        # recovery_alpha_slow 注入的随机粒子一旦权重更高就会直接接管。
        #
        # 0.05m 栅格地图上 500~2000 个粒子不可能达到 5mm 精度，
        # 出现这种 σ 只能是塔缩，不是真的定位准。
        # ==========================================================
        if std_xy < self.collapse_cov_xy and std_yaw < self.collapse_cov_yaw:
            self._converged_since = None
            if self._collapse_recoveries < self.max_collapse_recoveries:
                self._collapse_recoveries += 1
                self.get_logger().error(
                    '⚠️ 检测到粒子塔缩：σ_xy=%.4fm σ_yaw=%.4f° 低于物理下限'
                    '（%.2fm 栅格 + 最少 500 粒子不可能这么准）。\n'
                    '   成因：静止时反复 nomotion 更新，对同一帧激光重复'
                    '重采样，粒子副本完全重合、方差被压到 0。\n'
                    '   后果：滤波器失去全部备选假设，随机恢复粒子一旦'
                    '胜出，位姿会瞬间跳到几米外。\n'
                    '   处理：重新撒粒子恢复多样性（第 %d/%d 次）。'
                    % (std_xy, math.degrees(std_yaw),
                       (self._map_dims[2] if self._map_dims else 0.05),
                       self._collapse_recoveries,
                       self.max_collapse_recoveries))
                self._start_time = self.get_clock().now()
                self._trigger_global_localization()
            elif not self._fake_converge_warned:
                self._fake_converge_warned = True
                self.get_logger().warn(
                    '粒子塔缩已重试 %d 次仍复现，不再自动恢复。'
                    '请检查地图质量或重启导航。'
                    % self.max_collapse_recoveries)
            return

        if std_xy > self.converge_cov_xy or std_yaw > self.converge_cov_yaw:
            self._converged_since = None
            return

        if self._converged_since is None:
            self._converged_since = now
            return

        held = (now - self._converged_since).nanoseconds / 1e9
        if held < self.converge_hold_sec:
            return

        # 收敛前先做「假收敛」校验：σ 小不等于位置对
        ok, why = self._check_pose_on_map()
        if not ok:
            if not self._fake_converge_warned:
                self._fake_converge_warned = True
                self.get_logger().warn(
                    '⚠️ 疑似「假收敛」：粒子云已经很集中（σ_xy=%.3fm），'
                    '但%s。\n'
                    '   σ 只代表粒子一致性，不代表位置正确。静止时缺少新'
                    '观测证伪错误共识，很容易出现这种情况。\n'
                    '   建议：缓慢原地转动小车（转动比走直线更能消除角度'
                    '歧义），或考虑重建一张覆盖完整的图。'
                    % (std_xy, why))
            # 不判定收敛，继续等待更可靠的共识
            self._converged_since = None
            return

        self.state = self.STATE_CONVERGED
        self._finished = True
        self.get_logger().info(
            '✅ 定位已收敛（用时 %.1fs）：位姿=%s σ_xy=%.3fm σ_yaw=%.1f°, '
            'nomotion 请求 %d 次；%s'
            % (elapsed, self._fmt_pose(), std_xy,
               math.degrees(std_yaw), self._nomotion_requests, why))

        if self.stop_nomotion_after_converge:
            self.get_logger().info(
                '已停止周期性 nomotion 更新（注意：AMCL 将不再重发 '
                'map→odom，静止时若 odom 有漂移会直接体现为定位飘）')
            self._timer.cancel()
        else:
            # ★ 不能 cancel 定时器。收敛后还要继续按
            #   nomotion_update_period_after_converge_sec 请求更新，
            #   让 AMCL 持续用最新激光修正 map→odom。
            #   _tick 里的 _finished 检查会提前返回，所以这里要把它复位，
            #   否则 _tick 第一行就 return，请求永远不会发出去。
            self._finished = False
            self.get_logger().info(
                '继续以 %.1fs 周期请求 nomotion 更新，'
                '保持 map→odom 随激光持续刷新（消耗很低）'
                % self.nomotion_period_after_converge)

    def _check_scan_health(self) -> None:
        """激光消失时立即告警一次 —— 没有激光，全局定位不可能收敛。

        这一条是实测踩出来的：桥接板 TCP 连着但不再转发字节，
        `/scan` 静默消失，AMCL 粒子云永远不会被淘汰，
        而旧版日志只会说"车可能没动"，完全误导排查方向。
        """
        if self._warned_no_scan or self._scan_ok():
            return
        self._warned_no_scan = True
        scan_topic = str(self.get_parameter('scan_topic').value)
        self.get_logger().error(
            '❌ 激光话题 %s %s —— AMCL 没有观测，全局定位不可能收敛。\n'
            '  先按这个顺序查雷达链路：\n'
            '   1) ros2 topic hz %s          # 确认有没有帧\n'
            '   2) pgrep -af ydlidar_node    # 驱动在不在（respawn 会自己重启）\n'
            '   3) ss -tnp | grep 8889       # 转接板是否 ESTABLISHED\n'
            '   4) 看 tcp_server 日志的「桥接统计」：连着但字节不涨 = 转接板挂了'
            % (scan_topic,
               '从未收到' if self._scan_count == 0 else '已停止',
               scan_topic))

    def _check_timeout(self, elapsed: float) -> None:
        if self.state == self.STATE_CONVERGED:
            return
        if elapsed < self.timeout_sec:
            return
        # 超时不等于定位失败：粒子云没收敛往往只是这一轮撒得不好。
        # 重撒一次粒子通常就能成功，所以先重试再放弃。
        # 地图压根没收到时重试没意义，直接走超时诊断。
        if (self._map_ready
                and self._timeout_retries_used < self.timeout_retries):
            self._timeout_retries_used += 1
            self.get_logger().warn(
                '⏱️ 已 %.0fs 未收敛，重新撒粒子重试（第 %d/%d 次）'
                % (elapsed, self._timeout_retries_used, self.timeout_retries))
            self._start_time = self.get_clock().now()
            self._converged_since = None
            self._last_nomotion_time = self._start_time
            self._trigger_global_localization()
            return
        self.state = self.STATE_TIMEOUT
        self._finished = True

        # 把所有阻塞项集中列出来，而不是猜一个原因
        problems = []
        if not self._map_ready:
            problems.append(
                '从未收到地图 %s —— 检查 nav2 map_server 是否 configure+activate、'
                '地图路径是否正确' % self.get_parameter('map_topic').value)
        if self._scan_count == 0:
            problems.append(
                '从未收到激光 %s —— AMCL 没有观测，不可能收敛。'
                '优先查雷达链路（ydlidar_node / tcp_server / 转接板）'
                % self.get_parameter('scan_topic').value)
        elif not self._scan_ok():
            problems.append(
                '激光 %s 已停止 %.1fs —— 雷达链路中途断了，'
                '查 tcp_server 的「桥接统计」与转接板连接'
                % (self.get_parameter('scan_topic').value, self._scan_age()))
        if not self._trigger_sent and not self._prior_sent:
            problems.append(
                'AMCL 服务 %s 一直不可用 —— AMCL 可能没起来，'
                '`ros2 lifecycle get /amcl` 看一下'
                % self._global_loc_client.srv_name)
        if not problems:
            problems.append(
                '激光与地图都正常，但粒子云没收敛。常见原因：'
                '(1) 小车全程完全静止 —— 请缓慢原地转动 30~60°/s；'
                '(2) 地图与现场差异过大（未知区过多、家具搬动、尺度错误）；'
                '(3) /scan 时间戳与 TF 不同步（ros2 run tf2_ros tf2_echo odom base_footprint）')

        self.get_logger().error(
            '⚠️ 自动全局定位在 %.0fs 内未收敛（阶段=%s，%s）：位姿=%s 协方差=%s\n'
            '可能原因：\n  - %s\n'
            '可手动在 RViz 点 2D Pose Estimate，或修正上述问题后重新启动导航'
            % (elapsed, self.state, self._scan_status_text(),
               self._fmt_pose(), self._fmt_cov(),
               '\n  - '.join(problems)))
        self._timer.cancel()

    # ============================================================
    # 工具
    # ============================================================
    def _fmt_pose(self) -> str:
        if self._latest_pose is None:
            return '(尚无)'
        x, y, yaw = self._latest_pose
        return '(%.2f, %.2f, %.1f°)' % (x, y, math.degrees(yaw))

    def _fmt_cov(self) -> str:
        if self._latest_cov is None:
            return '(尚无)'
        var_x, var_y, var_yaw = self._latest_cov
        return '(σ_xy=%.3fm, σ_yaw=%.1f°)' % (
            math.sqrt(max(0.0, max(var_x, var_y))),
            math.degrees(math.sqrt(max(0.0, var_yaw))))

    def _log_periodically(self, message: str) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_log_time).nanoseconds / 1e9
        if dt < self.log_period:
            return
        self._last_log_time = now
        self.get_logger().info(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
