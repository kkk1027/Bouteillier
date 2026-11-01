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

import json
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty

from fishbot_navigation2.loc_quality import (
    LaserMapMatcher, MapGrid, PoseStabilityTracker, QualityAssessor,
    StatusSnapshot,
)

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
        # 超时时若 σ 已在收敛线 N 倍以内，视为「已很接近」，不重撒
        # （重撒会把本来正确的位姿扔掉 → 米级瞬移 → 有规律乱飘）
        self.declare_parameter('almost_converge_scale', 2.5)
        # ==========================================================
        # 收敛后是否还刷新 map→odom（2026-10-31 二次修订）
        # ----------------------------------------------------------
        # 曾经默认 True（收敛后完全停发）—— 这是个 bug 源头，已废弃使用。
        #
        # 为什么不能停：AMCL 只在**处理激光帧时**才重发 map→odom。
        # 本项目雷达经 WiFi 传输，实测每分钟约丢一帧，有效帧率 0.86Hz。
        # 一旦停发，nav2 controller_server（100ms 查一次 TF）就会报
        #   Lookup would require extrapolation into the future
        # 实测回起点过程中出现 5 次，其中一次直接把跟出一条控制周期打挂
        # （Controller patience exceeded → follow_path Aborting）。
        #
        # 现在改由 good_enough_cov_xy/yaw 统一控制：
        #   σ 低于够用线 → 自动暂停（防塔缩）；σ 涨回来 → 自动恢复。
        # 两个参数仅为向后兼容而保留，不再影响行为。
        self.declare_parameter('stop_nomotion_after_converge', False)
        self.declare_parameter('nomotion_update_period_after_converge_sec', 1.0)
        # 粒子塔缩判据：σ 低于物理下限即认为滤波器已退化。
        # 0.05m 栅格 + 最少 500 个粒子不可能达到 5mm 精度。
        self.declare_parameter('collapse_cov_xy', 0.005)
        self.declare_parameter('collapse_cov_yaw', 0.01)
        self.declare_parameter('max_collapse_recoveries', 3)
        # ==========================================================
        # ★ “够用即停”阈值（2026-10-31 静止塔缩复盘）
        # ----------------------------------------------------------
        # 原来的逻辑是“只要没进 CONVERGED 状态就一直发 nomotion”，
        # 而 CONVERGED 要求 σ_xy ≤ 0.08 且**持续** 3s。实测 σ 会卡在
        # 0.0446 上下抖动（CONVERGE↔DIVERGE 反复跳，hold 永远凑不满），
        # 于是 nomotion 一直发下去，对同一帧激光反复重采样，
        # σ 被单调压到 0.0010m —— 而 0.0446m 早就远优于导航需求了。
        #
        # 这里加一个“够用线”：σ 到 3.5cm（< 1 个栅格）后就不再请求
        # nomotion，让粒子云自然冻结。宁可不更新，也不能反复重采样。
        # ==========================================================
        self.declare_parameter('good_enough_cov_xy', 0.035)
        self.declare_parameter('good_enough_cov_yaw', 0.03)
        # ==========================================================
        # ★ 塔缩恢复方式（2026-10-31）
        # ----------------------------------------------------------
        # local  ：在当前位姿原位加劫动重撒 —— 恢复粒子多样性但**不瞬移**
        # global ：调 /reinitialize_global_localization 撒满全图 —— 必然瞬移
        #
        # 为什么默认改 local：
        #   塔缩时当前位姿其实**是对的**（实测车静止 60s 一直钉在
        #   (0.340,-0.120)，σ 才被压到 0.001）。错的只是“粒子云退化
        #   成单一副本、失去备选假设”。用全局重撒去救，等于把正确
        #   答案撕掉重猜 —— 实测直接跳了 1.66m。
        # 什么时候才用 global：
        #   真的被绑架（被人抱起换位置）/ 开局收敛阶段。
        # ==========================================================
        self.declare_parameter('collapse_recovery_mode', 'local')
        self.declare_parameter('local_reseed_cov_xy', 0.05)
        self.declare_parameter('local_reseed_cov_yaw', 0.03)
        # 稳定收敛多久后重置恢复配额（C）。
        # 原来 _collapse_recoveries 只增不减，3 次后永久放弃恢复。
        self.declare_parameter('collapse_rearm_sec', 15.0)
        # 塔缩恢复冷却期。
        # ★ 必需：重撒后要等 AMCL 回一帧新 /amcl_pose，_latest_cov 才会
        #   更新。tick_period 是 0.5s，而 AMCL 响应通常要 1~2s，这期间
        #   _latest_cov 还是旧的塌缩值，不加冷却就会每 0.5s 重撒一次、
        #   几秒内把 3 次配额全部烧光。
        self.declare_parameter('collapse_cooldown_sec', 4.0)
        self.declare_parameter('defer_on_external_initial_pose', True)
        self.declare_parameter('log_period_sec', 10.0)

        # ==========================================================
        # ★ 定位质量评估（2026-11-01）
        # ----------------------------------------------------------
        # 仅靠 σ 判收敛必然两头出错：
        #   真收敛时 σ 可能因为粒子略散而超线 → 永远不收敛 → 120s 重撒
        #   假收敛时 σ 很小但位置错，反而被当成功
        # 现在加入「激光-地图匹配率」这个物理证据，σ 降为参考项。
        # 详见 loc_quality.py 顶部的完整复盘。
        # ==========================================================
        # 地图障碍物膨胀半径（格）。2 格 = 10cm，容忍地图离散化与
        # 定位小偏差。
        self.declare_parameter('match_inflate_cells', 2)
        # 匹配率门限：
        #   good —— 达到此值 + 位姿稳定，即便 σ 超线也判收敛
        #   min  —— σ 达标时，匹配率至少要有这么高
        #   bad  —— 低于此值一票否决（无论 σ 多小）
        self.declare_parameter('match_good_rate', 0.75)
        self.declare_parameter('match_min_rate', 0.55)
        self.declare_parameter('match_bad_rate', 0.40)
        # 匹配率需要的最少有效束数 / 最少有效占比，不足则认为统计不可信
        self.declare_parameter('match_min_samples', 40)
        self.declare_parameter('match_min_coverage', 0.20)
        # 位姿稳定性：连续 N 秒内位移 < move_tol 且转角 < rot_tol_deg
        # 这就是用户说的「地图上的车头图标不动了」的代码化。
        self.declare_parameter('stable_window_sec', 3.0)
        self.declare_parameter('stable_move_tol_m', 0.05)
        self.declare_parameter('stable_rot_tol_deg', 2.0)
        # 激光相对 base_footprint 的平面偏移。
        # 本项目 URDF: laser_joint xyz="0 0 0.075" → x/y 都是 0。
        self.declare_parameter('laser_offset_x', 0.0)
        self.declare_parameter('laser_offset_y', 0.0)
        # 定位状态话题（JSON，供网页状态条与导航闸门使用）
        self.declare_parameter('status_topic', '/localization_status')
        self.declare_parameter('status_period_sec', 0.5)

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
        # 注：stop_nomotion_after_converge / nomotion_update_period_after_
        # converge_sec 两个参数仅为向后兼容而保留声明（旧 launch 可能还在
        # 传），实际行为已改由 good_enough_cov_xy/yaw 自适应控制。
        self.timeout_retries = int(
            self.get_parameter('timeout_retries').value)
        self.almost_converge_scale = max(
            1.0, float(self.get_parameter('almost_converge_scale').value))
        self.collapse_cov_xy = float(
            self.get_parameter('collapse_cov_xy').value)
        self.collapse_cov_yaw = float(
            self.get_parameter('collapse_cov_yaw').value)
        self.max_collapse_recoveries = int(
            self.get_parameter('max_collapse_recoveries').value)
        self.good_enough_cov_xy = float(
            self.get_parameter('good_enough_cov_xy').value)
        self.good_enough_cov_yaw = float(
            self.get_parameter('good_enough_cov_yaw').value)
        self.collapse_recovery_mode = str(
            self.get_parameter('collapse_recovery_mode').value).strip().lower()
        self.local_reseed_cov_xy = float(
            self.get_parameter('local_reseed_cov_xy').value)
        self.local_reseed_cov_yaw = float(
            self.get_parameter('local_reseed_cov_yaw').value)
        self.collapse_rearm_sec = float(
            self.get_parameter('collapse_rearm_sec').value)
        self.collapse_cooldown_sec = float(
            self.get_parameter('collapse_cooldown_sec').value)
        self.defer_on_external = bool(
            self.get_parameter('defer_on_external_initial_pose').value)
        self.log_period = float(self.get_parameter('log_period_sec').value)
        self.scan_stale_sec = float(
            self.get_parameter('scan_stale_sec').value)

        # ---------------- 定位质量评估（2026-11-01） ----------------
        self._assessor = QualityAssessor(
            cov_xy=self.converge_cov_xy,
            cov_yaw=self.converge_cov_yaw,
            good_rate=float(self.get_parameter('match_good_rate').value),
            min_rate=float(self.get_parameter('match_min_rate').value),
            bad_rate=float(self.get_parameter('match_bad_rate').value),
        )
        self._matcher = LaserMapMatcher(
            min_samples=int(self.get_parameter('match_min_samples').value),
            min_coverage=float(self.get_parameter('match_min_coverage').value),
        )
        self._stability = PoseStabilityTracker(
            window_sec=float(self.get_parameter('stable_window_sec').value),
            move_tol=float(self.get_parameter('stable_move_tol_m').value),
            rot_tol_deg=float(self.get_parameter('stable_rot_tol_deg').value),
        )
        self._inflate_cells = int(
            self.get_parameter('match_inflate_cells').value)
        self._laser_offset_x = float(
            self.get_parameter('laser_offset_x').value)
        self._laser_offset_y = float(
            self.get_parameter('laser_offset_y').value)
        self._status_topic = str(
            self.get_parameter('status_topic').value)
        self._status_period = max(
            0.1, float(self.get_parameter('status_period_sec').value))
        self._grid = None            # MapGrid，收到地图后构建
        self._latest_scan = None     # 最新一帧 LaserScan（匹配用）
        self._match = None           # MatchResult
        self._decision = None        # Decision
        self._last_status_time = None
        self._post_converge_low = 0  # 收敛后匹配率连续偏低计数

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
        # 首次进入 CONVERGED 的时刻，用于「稳定多久后重置恢复配额」（C）
        self._converged_at = None
        # 是否已进入「够用即停」：σ 足够小后不再发 nomotion（A）
        self._nomotion_suspended = False
        # 上次塔缩恢复的时刻（冷却期用，见 collapse_cooldown_sec）
        self._last_collapse_time = None
        # 是否已就「粒子好但地图校验不过」告警过
        self._blocked_by_map_warned = False
        # “已很接近收敛所以不重撒”的提示已打印次数（限流）
        self._nearly_converged_reported = 0
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
        # 定位状态（JSON 文本）。网页状态条和导航闸门都读它，
        # 这样用户永远不需要问「好了没」。
        self._status_pub = self.create_publisher(
            String, self._status_topic, 10)
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

        # 构建匹配率用的栅格（带障碍物膨胀），只做一次
        try:
            self._grid = MapGrid(
                msg.data, width, height, res, origin.x, origin.y,
                inflate_cells=self._inflate_cells)
            self.get_logger().info(
                '激光-地图匹配器就绪：%dx%d 栅格，障碍物膨胀 %d 格（%.0fcm）'
                % (width, height, self._inflate_cells,
                   self._inflate_cells * res * 100))
        except Exception as exc:                   # noqa: BLE001
            self._grid = None
            self.get_logger().warn(
                '构建匹配栅格失败，将退化为仅用 σ 判断收敛: %s' % exc)

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
        # 缓存最新一帧供匹配率计算。471 束 ≈ 4KB，留一帧足够。
        self._latest_scan = msg
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

        # ★ 定位状态每 tick 都尝试发布（内部按 status_period 限流）。
        #   放在最前面是为了让网页状态条在等待地图/服务阶段也有内容，
        #   而不是一片空白。
        self._publish_status()

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
            # 先算匹配率，再判收敛（匹配率是主判据）
            self._compute_match()
            self._evaluate_convergence(now, elapsed)
            if self._finished:      # 本轮已收敛，不再打"进行中"
                return
            self._log_periodically(
                '全局定位进行中（已 %.0fs，%s）：位姿=%s 协方差=%s %s'
                % (elapsed, self._scan_status_text(),
                   self._fmt_pose(), self._fmt_cov(), self._fmt_match()))
            return

        # ---------- 阶段 3：已收敛，继续盯塔缩 ----------
        # ★ P0（2026-10-31）：收敛**不等于**可以不再刷新 map→odom。
        #   nav2 controller_server 每 100ms 查一次 TF，而 AMCL 只在处理
        #   激光帧时才重发。雷达经 WiFi 传输、有效帧率 0.86Hz，停发就会
        #   让 TF 间隔超过 1s，触发 extrapolation into the future，
        #   实测导致 Controller patience exceeded / 跟出被打断。
        #   现在由 _maybe_request_nomotion 内部的「σ 够用即停」自适应：
        #   静止时 σ 自动降到够用线以下而停发，一动就恢复。
        if self.state == self.STATE_CONVERGED:
            self._maybe_request_nomotion(now)
            # ★ B：收敛不等于没事了。AMCL 仍在消费 /scan，一旦 odom 抖动
            #   超标就会更新→重采样→方差塔缩→内置 recovery 注入随机粒子
            #   →位姿瞬移。必须全程盯住，不能收敛就撒手。
            if self._check_collapse(now):
                return
            # ★ C：稳定足够久就把恢复配额用回来，避免“3 次后永久放弃”
            self._maybe_rearm_collapse_recoveries(now)
            # 收敛后仍然刷新匹配率，让网页状态条始终显示真实质量，
            # 也便于发现「被抱走」这类突发情况。
            self._compute_match()
            self._check_post_converge_health()

    # ============================================================
    # 收敛后的健康监控
    # ============================================================
    def _check_post_converge_health(self) -> None:
        """收敛后匹配率突然崩掉 → 大概率被抱走 / 地图被改。

        只告警不自动改状态：导航可能正在进行中，贸然把 state 改回
        CONVERGING 会让导航闸门在任务中途拒绝后续下单，反而更糟。
        """
        match = self._match
        if match is None or match.rate is None:
            return
        if match.rate >= self._assessor.bad_rate:
            self._post_converge_low = 0
            return
        self._post_converge_low = getattr(self, '_post_converge_low', 0) + 1
        # 连续 3 次（约 1.5s）都很低才报，避免单帧噪声
        if self._post_converge_low == 3:
            self.get_logger().warn(
                '⚠️ 收敛后激光匹配率降到 %.0f%%（低于否决线 %.0f%%）。'
                '小车可能被移动过，或地图已被修改。\n'
                '   如果小车确实被挪动，请重新「载入地图」触发全局定位。'
                % (match.rate * 100, self._assessor.bad_rate * 100))

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

    def _publish_local_reseed(self) -> bool:
        """在**当前位姿附近**局部重撒粒子 —— 恢复多样性但不瞬移。

        ★ 为什么不能用全局重撒：
          `/reinitialize_global_localization` 会把粒子云均匀撒满整张
          地图，原来已经收敛好的位姿彻底丢失，必须重新收敛。实测
          一次重撒就让位姿瞬移 1.66m，而那一刻车根本没动。

        ★ 塔缩时位姿为什么是对的：
          塌缩的成因是「静止时反复对同一帧激光重采样，副本彼此重合，
          方差单调压到 0」—— 被压掉的是**不确定性**，不是**估计值**。
          实测车静止 60s，位姿一直钉在 (0.340, -0.120)，σ 才被压到
          0.0010m。所以位姿可信，只是滤波器失去了全部备选假设。

        ★ 做法：
          以当前位姿为中心发一帧 `/initialpose`，协方差给到
          local_reseed_cov_xy（默认 0.05m，≈1 个栅格）。AMCL 会就地
          重新散布粒子，既恢复了多样性，又不会跳到别处。

        Returns:
            True 表示重撒成功，False 表示没有可用位姿。
        """
        if self._latest_pose is None:
            self.get_logger().warn(
                '未收到过 /amcl_pose，无法原位重撒；降级为全局重撒')
            return False
        x, y, yaw = self._latest_pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[COV_X] = self.local_reseed_cov_xy
        cov[COV_Y] = self.local_reseed_cov_xy
        cov[COV_YAW] = self.local_reseed_cov_yaw
        msg.pose.covariance = cov
        # 发两帧，避免 AMCL 刚好在处理激光而丢掉
        for _ in range(2):
            self._initial_pose_pub.publish(msg)
        # 自环过滤：这两帧会被自己的订阅收回去，必须认领
        self._self_prior_stamp = self.get_clock().now()
        self._self_prior_xy = (x, y)
        self.get_logger().warn(
            '已在**原位**重撒粒子：x=%.2f y=%.2f yaw=%.0f°，'
            'σ_xy=%.2fm σ_yaw=%.3frad'
            '（不改变当前位姿估计，仅恢复粒子多样性）'
            % (x, y, math.degrees(yaw),
               self.local_reseed_cov_xy, self.local_reseed_cov_yaw))
        return True

    def _cov_below_good_enough(self) -> bool:
        """协方差是否已小到「够用」，不必再请求 nomotion 刷新。"""
        if self._latest_cov is None:
            return False
        var_x, var_y, var_yaw = self._latest_cov
        std_xy = math.sqrt(max(0.0, max(var_x, var_y)))
        std_yaw = math.sqrt(max(0.0, var_yaw))
        return (std_xy <= self.good_enough_cov_xy
                and std_yaw <= self.good_enough_cov_yaw)

    def _cov_below_converge(self, scale: float = 1.0) -> bool:
        """协方差是否已达到**收敛判据**（但可能被地图校验卡住）。

        用于区分「粒子云真的没收敛」与「粒子云很好但地图校验不过」：
        前者需要重撒，后者绝不能重撒（会把正确位姿扔掉）。

        Args:
            scale: 放宽倍数。scale=2.5 表示只要 σ 在收敛线 2.5 倍以内
                就算「已经很接近」，此时重撒同样是灾难。
        """
        if self._latest_cov is None:
            return False
        var_x, var_y, var_yaw = self._latest_cov
        return (math.sqrt(max(0.0, max(var_x, var_y)))
                <= self.converge_cov_xy * scale
                and math.sqrt(max(0.0, var_yaw))
                <= self.converge_cov_yaw * scale)

    def _maybe_request_nomotion(self, now) -> None:
        # ==========================================================
        # 什么时候需要请求 nomotion 更新
        # ----------------------------------------------------------
        # AMCL 只在**处理到激光帧时**才会重发 map→odom 这个 TF。
        # 小车静止时没有运动，AMCL 默认不更新粒子，也就不重发 TF；
        # 而 nav2 的 controller_server / planner_server 每 100ms 就要查
        # 一次 map→odom，查不到就报 extrapolation into the future。
        #
        # ==========================================================
        # 统一闸门：σ 够用就停，涨回来就恢复（2026-10-31 二次修订）
        # ----------------------------------------------------------
        # 原来这里分两种情况：
        #   CONVERGED  → 若 stop_nomotion_after_converge 则完全不发
        #   其它       → 按 good_enough_cov_xy 暂停
        #
        # 「收敛后完全停发」是个 bug 源头。本项目雷达经 WiFi 传输、
        # 实测还有约每分钟一次丢帧，有效帧率仅 0.86Hz —— 一旦停发，
        # TF 间隔必然超过 1 秒。
        #
        # 实测后果（2026-10-31 21:45 回起点过程）：
        #   Exception in transformPose: Lookup would require extrapolation
        #   into the future. Requested 1793454305.734 but latest data is
        #   1793454304.859785            ← 滞后 0.87s
        #   Unable to transform robot pose into global plan's frame
        #   Controller patience exceeded → [follow_path] Aborting handle
        # 共 5 次，其中一次直接把跟出一条控制周期打挂。
        #
        # 现在改成：**不看状态，只看 σ**。
        #   σ < 够用线 → 暂停（粒子云冻结，避免反复重采样造成塔缩）
        #   σ 涨回来   → 恢复低频刷新（实时补上 TF）
        # 静止时 σ 自己会往下走、自动停；一动 σ 立刻涨起来、自动续上，
        # 不需要人判断「收敛了没有」。
        # ==========================================================
        if self._cov_below_good_enough():
            if not self._nomotion_suspended:
                self._nomotion_suspended = True
                self.get_logger().info(
                    'σ 已小于“够用线”(%.3fm/%.3frad)，暂停请求 '
                    'nomotion 更新，冻结粒子云以免反复重采样造成塔缩'
                    % (self.good_enough_cov_xy,
                       self.good_enough_cov_yaw))
            return
        if self._nomotion_suspended:
            # σ 又变大了（车移动 / 局部重撒后），恢复请求
            self._nomotion_suspended = False
            self._last_nomotion_time = now
            self.get_logger().info(
                'σ 已超过“够用线”，恢复请求 nomotion 更新（维持 map→odom 新鲜）')
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
        """校验当前位姿是否落在合理位置。

        ==========================================================
        ★ 2026-11-01 重写：从「单像素硬否决」改为「邻域 + 软硬区分」
        ==========================================================
        旧实现只看**一个 5cm 栅格**：
            val = self._map_data[row * width + col]
            if val < 0:  return False, '位姿落在未知区域'
        两个致命问题：

        1. 这张图 **67.9% 是未知区**，位姿那一个格子很容易落在灰色
           区域。于是正确位姿被反复判「假收敛」→ 永远不收敛 →
           120s 超时重撒 → 位姿全图乱飘。本次事故的主线就是这个。
        2. 单像素没有冗余，边界噪声一来判定就翻转。

        新实现：
          * 查 (2*radius+1)² 邻域，只要有**任意一格**是已知空闲就算通过
            —— 容忍地图离散化与定位小偏差。
          * 区分「软失败」与「硬失败」：
              硬失败（物理不可能）：落在墙上 / 地图外 → 仍然否决
              软失败（只是图不全）  ：邻域全是未知区   → 只提示不否决
            因为现在有激光匹配率这个更强的物理证据，地图覆盖不全
            不该再一票否决正确位姿。

        Returns:
            (是否可信, 说明字符串, 是否硬失败)
        """
        if self._map_data is None or self._map_dims is None:
            return True, '（无地图数据，跳过校验）', False
        if self._latest_pose is None:
            return True, '（无位姿）', False

        width, height, res, ox, oy = self._map_dims
        x, y, _ = self._latest_pose
        # ==========================================================
        # ★ 世界坐标 -> OccupancyGrid 索引（2026-10-31 修过一个致命 bug）
        # ----------------------------------------------------------
        # 这里**不能**做 Y 翻转！
        #
        # `_map_data` 来自 /map（OccupancyGrid），而 map_server 加载 PGM 时
        # 已经翻过一次：data[x + (height - y_img - 1) * width]。也就是说
        # OccupancyGrid 的 data 行号就是**世界坐标系自下往上**的行号，
        # 直接等于 int((y - origin_y) / res)。
        #
        # 旧代码照搬了「直接读 PGM 文件」的写法，又翻了一次：
        #     row = height - 1 - int((y - oy) / res)
        # 结果读到的是**上下镜像**的像素。
        # ==========================================================
        col = int((x - ox) / res)
        row = int((y - oy) / res)

        if not (0 <= col < width and 0 <= row < height):
            return False, '位姿在地图范围之外 (%.2f, %.2f)' % (x, y), True

        radius = 2
        saw_free = False
        saw_occupied = False
        saw_unknown = False
        for dy in range(-radius, radius + 1):
            r = row + dy
            if not (0 <= r < height):
                continue
            base = r * width
            for dx in range(-radius, radius + 1):
                c = col + dx
                if not (0 <= c < width):
                    continue
                val = self._map_data[base + c]
                if val < 0:
                    saw_unknown = True
                elif val >= 65:
                    saw_occupied = True
                else:
                    saw_free = True

        if saw_free:
            return (True,
                    '位姿位于已知空闲区域（栅格 %d,%d）' % (col, row), False)
        if saw_occupied:
            return (False, '位姿落在障碍物上（物理上不可能）', True)
        return (False, '位姿邻域全是未知区域（地图没覆盖这里）', False)

    def _check_collapse(self, now) -> bool:
        """检测并处理粒子塔缩。返回 True 表示当前正处于塔缩状态。

        抽成独立方法是 2026-10-31 的修订：原来它埋在
        `_evaluate_convergence` 里，而那个函数**只在 CONVERGING 状态
        被调用**。一旦收敛就会 cancel 定时器，节点彻底停眼 —— 之后
        AMCL 再怎么塔缩、再怎么被 recovery 注入随机粒子都不再有人管，
        实测两次 1.4m+ 的瞬移就这样静默发生了。
        """
        if self._latest_cov is None:
            return False
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
        # 实测证据（2026-10-31 tracer 抓的 127s 轨迹）：
        #   σ_xy 稳定在 0.0446m → 0.0234 → 0.0010 → 位姿跳 1.46m
        # 两次完全一样的循环，周期 68s，可重复。
        #
        # 0.05m 栅格地图上 500~2000 个粒子不可能达到 5mm 精度，
        # 出现这种 σ 只能是塔缩，不是真的定位准。
        # ==========================================================
        if not (std_xy < self.collapse_cov_xy
                and std_yaw < self.collapse_cov_yaw):
            return False

        # ★ 冷却期：重撒后 _latest_cov 要等 AMCL 下一帧 /amcl_pose 才更新，
        #   这期间它还是旧的塌缩值。不加冷却会每 tick 重撒一次。
        if self._last_collapse_time is not None:
            since = (now - self._last_collapse_time).nanoseconds / 1e9
            if since < self.collapse_cooldown_sec:
                return True

        self._converged_since = None
        self._converged_at = None
        if self._collapse_recoveries < self.max_collapse_recoveries:
            self._collapse_recoveries += 1
            self._last_collapse_time = now
            self.get_logger().error(
                '⚠️ 检测到粒子塔缩：σ_xy=%.4fm σ_yaw=%.4f° 低于物理下限'
                '（%.2fm 栅格 + 最少 500 粒子不可能这么准）。\n'
                '   成因：静止时反复更新，对同一帧激光重复重采样，'
                '粒子副本完全重合、方差被压到 0。\n'
                '   后果：滤波器失去全部备选假设，随机恢复粒子一旦'
                '胜出，位姿会瞬间跳到几米外。\n'
                '   处理：%s（第 %d/%d 次）。'
                % (std_xy, math.degrees(std_yaw),
                   (self._map_dims[2] if self._map_dims else 0.05),
                   ('原位局部重撒，保持当前位姿'
                    if self.collapse_recovery_mode == 'local'
                    else '全局重撒，粒子云撒满全图'),
                   self._collapse_recoveries,
                   self.max_collapse_recoveries))
            self._start_time = self.get_clock().now()
            self.state = self.STATE_CONVERGING
            self._nomotion_suspended = False
            # ★ B：优先原位重撒。塔缩时位姿是对的，只剩多样性丢了，
            #   局部微扰就够；全局重撒会把正确位姿直接抹掉造成瞬移。
            if self.collapse_recovery_mode == 'local':
                if not self._publish_local_reseed():
                    self._trigger_global_localization()
            else:
                self._trigger_global_localization()
        elif not self._fake_converge_warned:
            self._fake_converge_warned = True
            self.get_logger().warn(
                '粒子塔缩已重试 %d 次仍复现，不再自动恢复。'
                '请检查地图质量或重启导航。'
                % self.max_collapse_recoveries)
        return True

    def _maybe_rearm_collapse_recoveries(self, now) -> None:
        """C：稳定收敛足够久后，把塔缩恢复配额用回来。

        原来 `_collapse_recoveries` 只增不减，3 次之后永久放弃恢复
        （只 warn 一次），第 4 次塔缩起就只能带着退化粒子云硬跑。
        但一个正常的长时间导航任务里，小车走走停停，停在原地
        十几秒就塔缩一次是很常见的，3 次配额根本不够用。
        """
        if self._collapse_recoveries <= 0 or self._converged_at is None:
            return
        stable = (now - self._converged_at).nanoseconds / 1e9
        if stable < self.collapse_rearm_sec:
            return
        self.get_logger().info(
            '定位稳定 %.0fs 无塔缩，重置塔缩恢复配额（%d → 0）'
            % (stable, self._collapse_recoveries))
        self._collapse_recoveries = 0
        self._fake_converge_warned = False

    def _evaluate_convergence(self, now, elapsed: float) -> None:
        if self._latest_cov is None:
            return
        # 塔缩优先处理：它会让 σ 假性变小，必须先排除再谈收敛
        if self._check_collapse(now):
            return

        var_x, var_y, var_yaw = self._latest_cov
        std_xy = math.sqrt(max(0.0, max(var_x, var_y)))
        std_yaw = math.sqrt(max(0.0, var_yaw))

        # ---- 位姿稳定性（B）：把"地图上的车头图标不动了"写成代码 ----
        stable, stable_sec = self._stability.update(
            self._elapsed(), self._latest_pose)

        # ---- 激光-地图匹配率（A）：471 束激光共同投票 ----
        match = self._match
        if match is None:
            match = self._compute_match()

        # ---- 综合判定（C：σ 从硬门槛降为参考项） ----
        decision = self._assessor.assess(
            std_xy, std_yaw, match, stable, stable_sec)
        self._decision = decision

        if not decision.converged:
            self._converged_since = None
            # ★ 物理否决时给出一次性告警：这种情况是"位姿真的错了"，
            #   和"再等等"完全不同，必须让用户看见。
            if decision.veto and not self._fake_converge_warned:
                self._fake_converge_warned = True
                self.get_logger().warn(
                    '⚠️ %s\n   这与「收敛慢」是两回事：粒子云可能已经'
                    '收敛到一个错误位置。\n   建议缓慢原地转动小车'
                    '（转动比走直线更能消除角度歧义）。'
                    % decision.reason)
            return

        if self._converged_since is None:
            self._converged_since = now
            return

        held = (now - self._converged_since).nanoseconds / 1e9
        if held < self.converge_hold_sec:
            return

        # 收敛前再确认一次「位姿没有落在障碍物上」。
        #
        # ★ 2026-11-01 修订：这一步从「硬否决」降级为「只否决物理上
        #   不可能的情况」。
        #   原来它只查一个 5cm 栅格，位姿落在那一个格子是未知区就否掉，
        #   而这张图 67.9% 是未知区 —— 于是正确位姿被反复否掉，
        #   永远不收敛 → 120s 重撒 → 位姿全图乱飘（本次事故主线）。
        #   现在「落在未知区」只记一次提示，不再否决；
        #   真正否决的只剩「落在墙上/地图外」这种物理不可能。
        ok, why, hard = self._check_pose_on_map()
        if not ok:
            if hard:
                if not self._fake_converge_warned:
                    self._fake_converge_warned = True
                    self.get_logger().warn(
                        '⚠️ 位姿物理上不成立：%s。不判定收敛。' % why)
                self._converged_since = None
                return
            if not self._blocked_by_map_warned:
                self._blocked_by_map_warned = True
                self.get_logger().info(
                    '位姿落在未知区域（%s），但激光匹配率%s，'
                    '判定可信 —— 这只是地图覆盖不全，不影响导航'
                    % (why,
                       '不可用' if match is None or match.rate is None
                       else '%.0f%%' % (match.rate * 100)))

        self.state = self.STATE_CONVERGED
        self._converged_at = now
        self.get_logger().info(
            '✅ 定位已收敛（用时 %.1fs）：位姿=%s σ_xy=%.3fm σ_yaw=%.1f°, '
            '匹配率=%s，%s；nomotion 请求 %d 次'
            % (elapsed, self._fmt_pose(), std_xy,
               math.degrees(std_yaw),
               '不可用' if match is None or match.rate is None
               else '%.0f%%（%d/%d）'
               % (match.rate * 100, match.hit, match.hit + match.miss),
               decision.reason, self._nomotion_requests))

        # ==========================================================
        # ★ 收敛后**不再终止节点**（2026-10-31 修订）
        # ----------------------------------------------------------
        # 原来这里设 _finished=True 并 _timer.cancel()，节点彻底停眼。
        # 但 tracer 实测发现：收敛后 AMCL 依然会因 odom 微小抖动更新并
        # 重采样，σ 被压到 0.0010m，随后 AMCL 内置 recovery
        # （recovery_alpha_slow 当时是激进的 0.01）注入随机粒子，
        # 位姿静默瞬移 1.46m —— 而没有任何代码在看者。
        #
        # 现在改成：保留定时器，继续以 tick_period 监控。同时不停止
        # nomotion（否则 map→odom 会陈旧到让 controller 报 TF 外推失败），
        # 改由「σ 够用即停」自适控制。
        # ==========================================================
        self.get_logger().info(
            '收敛后继续监控：σ 涨过够用线(%.3fm)就恢复 nomotion 刷新 '
            'map→odom，σ 降下去就自动暂停粒子云'
            % self.good_enough_cov_xy)

    # ============================================================
    # 定位质量评估（2026-11-01）
    # ============================================================
    def _compute_match(self):
        """用当前位姿 + 最新激光算一次「激光-地图匹配率」。

        为什么值得每 0.5s 算一遍：
          这是唯一能区分「σ 小但位置错」的物理证据。471 束激光一起
          投票，比 σ 那种"粒子自己跟自己一致"的指标可靠得多。
          代价：一次遍历 471 束（大多数只做 1 次栅格查找），
          实测在 2Hz 下可忽略。
        """
        self._match = None
        if self._grid is None or self._latest_pose is None:
            return None
        scan = self._latest_scan
        if scan is None or not self._scan_ok():
            return None
        try:
            self._match = self._matcher.evaluate(
                self._grid, self._latest_pose, scan.ranges,
                angle_min=float(scan.angle_min),
                angle_increment=float(scan.angle_increment),
                range_min=float(scan.range_min),
                range_max=float(scan.range_max),
                laser_offset_x=self._laser_offset_x,
                laser_offset_y=self._laser_offset_y,
            )
        except Exception as exc:                   # noqa: BLE001
            self.get_logger().warn('匹配率计算失败（忽略本轮）: %s' % exc)
            self._match = None
        return self._match

    # --------------------------------------------------------
    def _build_status(self) -> StatusSnapshot:
        """汇总成一份可发布的定位状态。"""
        var_x = var_y = var_yaw = None
        if self._latest_cov is not None:
            var_x, var_y, var_yaw = self._latest_cov

        std_xy = None
        std_yaw_deg = None
        if var_x is not None:
            std_xy = math.sqrt(max(0.0, max(var_x, var_y)))
            std_yaw_deg = math.degrees(math.sqrt(max(0.0, var_yaw)))

        match = self._match
        decision = self._decision
        # ★ 不在这里调 update()：_evaluate_convergence 已经用同一时刻
        #   的位姿喂过一次了，再喂一次会让窗口里出现重复样本。
        stable, stable_sec = self._stability.last

        snap = StatusSnapshot(
            state=self.state,
            ready=(self.state == self.STATE_CONVERGED),
            progress=(decision.progress if decision is not None else 0.0),
            reason=(decision.reason if decision is not None else ''),
            hint=(decision.hint if decision is not None else ''),
            std_xy=std_xy,
            std_yaw_deg=std_yaw_deg,
            sigma_ok=bool(decision.sigma_ok) if decision else
            (std_xy is not None and std_xy <= self.converge_cov_xy),
            stable=stable,
            stable_sec=stable_sec,
            scan_ok=self._scan_ok(),
        )
        if match is not None:
            snap.match_rate = match.rate
            snap.match_ok = bool(decision.match_ok) if decision else False
            snap.hit = match.hit
            snap.miss = match.miss
            snap.unknown = match.unknown
            snap.coverage = match.coverage
        if self._latest_pose is not None:
            px, py, pyaw = self._latest_pose
            snap.pose = {'x': round(px, 3), 'y': round(py, 3),
                         'yaw_deg': round(math.degrees(pyaw), 1)}
        snap.extra = {
            'elapsed': round(self._elapsed(), 1),
            'timeout_sec': self.timeout_sec,
            'inflate_cells': self._inflate_cells,
        }
        return snap

    def _publish_status(self) -> None:
        """发布 /localization_status（JSON）。

        网页状态条和 nav_manager 的导航闸门都靠它，这样
        用户永远不必反复问「定位好了没」。
        """
        now = self.get_clock().now()
        if self._last_status_time is not None:
            age = (now - self._last_status_time).nanoseconds / 1e9
            if age < self._status_period:
                return
        self._last_status_time = now
        try:
            payload = json.dumps(self._build_status().as_dict(),
                                 ensure_ascii=False)
            msg = String()
            msg.data = payload
            self._status_pub.publish(msg)
        except Exception as exc:                   # noqa: BLE001
            self.get_logger().warn('发布定位状态失败: %s' % exc)

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
        # ==========================================================
        # ★ 粒子云已经够集中时，绝不能再全局重撒（2026-10-31）
        # ----------------------------------------------------------
        # 之前只要「没进 CONVERGED」就无脑重撒。但没进 CONVERGED 有
        # ========================================================
        # ★ 粒子云已经够好时，绝不能再全局重撒（2026-10-31）
        # --------------------------------------------------------
        # 之前只要「没进 CONVERGED」就无脑重撒。但没进 CONVERGED
        # 有两种完全不同的原因：
        #   (a) 粒子云真的没收敛      → 重撒是对的
        #   (b) 粒子云已经很集中，只是**地图校验**不通过（例如位姿落在
        #       未知区）            → 重撒是灾难
        #
        # 情况 (b) 实测后果：σ_xy=0.027m 的极好估计被撒满全图，
        # 位姿瞬间跳 1.83m，然后重新收敛、再次超时、再次跳走 ——
        # 每 120 秒循环一次。用户看到的就是「车自己隔一会儿蹦一下」。
        #
        # 2026-11-01 补充：这个分支原先只挡 σ 达标的情况，
        # 而事故里 σ=0.112m 差 0.032m 就没被挡住，照样重撒乱飘。
        # 现在把「匹配率高 + 位姿稳定」也算进来 —— 物理证据足够强时
        # 同样不许重撒。见下面的 almost_converge_scale 分支。
        # ==========================================================
        if self._cov_below_converge():
            if not self._blocked_by_map_warned:
                self._blocked_by_map_warned = True
                _, why, _hard = self._check_pose_on_map()
                self.get_logger().error(
                    '⚠️ 粒子云已经很集中（σ_xy=%.3fm σ_yaw=%.1f°），却无法'
                    '判定收敛：%s\n'
                    '   已**不做**全局重撒 —— 那会把这个本来正确的位姿直接'
                    '扔掉，造成米级瞬移。\n'
                    '   当前 %s\n'
                    '   若地图未知区过多，建议重建一张覆盖完整的图。'
                    % (math.sqrt(max(0.0, max(self._latest_cov[0],
                                              self._latest_cov[1]))),
                       math.degrees(math.sqrt(max(0.0,
                                                  self._latest_cov[2]))),
                       why, self._fmt_match()))
            # 不重撒、不终止，继续盯着（若换图/移动后恢复正常会自动收敛）
            self._start_time = self.get_clock().now()
            self._converged_since = None
            return

        # ========================================================
        # ★ 粒子云「已经很接近收敛」时也不要重撒（2026-11-01）
        # --------------------------------------------------------
        # 实测（2026-11-01 事故日志）：
        #   σ_xy 从 1.50m 一路降到 0.11m（72s 时），但收敛线是 0.08m ——
        #   差 0.03m 就判失败，120s 一到把粒子云撒满全图，
        #   位姿瞬间全图乱跳，然后重新收敛、再次超时、再次跳走。
        #   用户看到的就是「有规律乱飘」。
        #
        # σ 已降到收敛线 2.5 倍以内，说明**定位基本是对的**，
        # 重撒只会把对的扔掉。此时改为延长截止时间（不重撒），
        # 给 AMCL 更多时间自然收敛。
        # ========================================================
        if self._latest_cov is not None and \
                self._cov_below_converge(scale=self.almost_converge_scale):
            if self._nearly_converged_reported < 3:
                self._nearly_converged_reported += 1
                self.get_logger().info(
                    '已 %.0fs 未达标，但粒子云已很集中'
                    '（σ_xy=%.3fm σ_yaw=%.1f°，门限 %.3fm），'
                    '**不重撒**，延长等待'
                    % (elapsed,
                       math.sqrt(max(0.0, max(self._latest_cov[0],
                                              self._latest_cov[1]))),
                       math.degrees(math.sqrt(max(0.0, self._latest_cov[2]))),
                       self.converge_cov_xy))
            self._start_time = self.get_clock().now()
            self._converged_since = None
            return

        # ========================================================
        # ★ 物理证据很强时同样不许重撒（2026-11-01 新增）
        # --------------------------------------------------------
        # 上一条只看 σ。但 σ 可能因为粒子略散而超线，而位姿其实一直是
        # 对的 —— 事故里 σ=0.112m / 收敛线 0.08m 就是这种情况，
        # 差 0.032m 没被上一条挡住，120s 一到照样重撒、照样乱飘。
        #
        # 现在加一条：**匹配率 ≥ good_rate 且位姿稳定**，说明激光与地图
        # 高度吻合、估计值也不再变化，这时重撒纯属破坏。
        # ========================================================
        match = self._match
        stable, _stable_sec = self._stability.last
        if (match is not None and match.rate is not None
                and match.rate >= self._assessor.good_rate and stable):
            if self._nearly_converged_reported < 3:
                self._nearly_converged_reported += 1
                self.get_logger().info(
                    '已 %.0fs 未达标，但激光匹配 %.0f%% 且位姿已稳定，'
                    '**不重撒**（σ 只是参考项，物理证据更可靠）'
                    % (elapsed, match.rate * 100))
            self._start_time = self.get_clock().now()
            self._converged_since = None
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

    def _fmt_match(self) -> str:
        """匹配率的一行描述。"""
        m = self._match
        if m is None:
            return '(匹配率：无激光/无地图)'
        if m.rate is None:
            return '(匹配率：样本不足 hit=%d miss=%d unk=%d)' % (
                m.hit, m.miss, m.unknown)
        return '(匹配率=%.0f%% 命中%d/%d, 覆盖率%.0f%%)' % (
            m.rate * 100, m.hit, m.hit + m.miss, m.coverage * 100)

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
