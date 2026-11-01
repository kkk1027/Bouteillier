"""
navigation_service.py
====================================================
统一导航服务层 — Web UI、AI Agent 与 Nav2 Stack 的唯一边界

三层架构：
┌────────────────────────────────────────────────────────┐
│  Web 层 (Flask + SocketIO)      AI 层 (LangGraph)      │
│       │                               │                │
│       └──────────┬────────────────────┘                │
│                  ▼                                     │
│         NavigationService  ◄── 唯一入口，线程安全       │
│                  │                                     │
│                  ▼                                     │
│           PatrolNode (BasicNavigator)                  │
│                  │                                     │
│                  ▼                                     │
│            Nav2 Stack (AMCL + SmacPlanner)              │
└────────────────────────────────────────────────────────┘

核心原则：
  1. ROS2 边界 → NavigationService 内部，不暴露给 Web/AI
  2. 线程边界 → 内部用锁保护，外部无锁调用
  3. 状态边界 → NavigationService 是唯一状态源
  4. 进程边界 → 不再用 subprocess 启动导航

设计决策：
  - PatrolNode 是单例，waitUntilNav2Active() 只调用一次
  - waitUntilNav2Active() 内部启动后台 spin 线程
  - 后续所有导航调用复用同一个 PatrolNode
  - 支持连续导航（不重复初始化）
"""

import json
import math
import os
import threading
import time
from enum import Enum
from typing import Optional, Callable

import rclpy
from rclpy.duration import Duration
from tf_transformations import euler_from_quaternion

# ============================================================
# 全局导航互斥锁：防止 Web UI 和 AI 同时发起导航
# ============================================================
from autopatrol_robot.nav_manager import get_global_nav_lock


# ============================================================
# 状态枚举
# ============================================================
class NavState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    CANCELED = "canceled"
    COMPLETED = "completed"
    FAILED = "failed"


# ============================================================
# NavigationService — 统一导航服务
# ============================================================
class NavigationService:
    """
    统一导航服务。Web UI 和 AI Agent 都通过此服务控制机器人导航。

    用法（Web 端）:
        nav_service = NavigationService(socketio=socketio)
        nav_service.navigate_to(1.0, 2.0, 0.0)

    用法（AI Agent 端）:
        nav_service = NavigationService()  # 无 socketio
        nav_service.navigate_to(1.0, 2.0, 0.0)

    线程安全: 所有 public 方法都是线程安全的。
    """

    def __init__(self, status_callback: Optional[Callable[[str, str], None]] = None):
        """
        初始化导航服务。

        Args:
            status_callback: 状态回调函数，签名为 callback(event_type, message)
                             例如用于向 Web 端推送 SocketIO 事件
        """
        self._lock = threading.Lock()
        self._patrol_node: Optional['PatrolNode'] = None
        self._state: NavState = NavState.IDLE
        self._nav2_ready = False
        self._status_callback = status_callback
        self._rclpy_initialized_here = False  # 是否由本服务初始化了 rclpy
        self._cancel_requested = False
        self._last_arrival_pose = None  # 最近一次导航到达时的实际位姿 (x,y,yaw)

        # ==========================================================
        # 独立的 /amcl_pose 订阅节点
        # ----------------------------------------------------------
        # 为什么不用 PatrolNode 里的订阅：
        #   BasicNavigator 的后台 spin 线程只在 waitUntilNav2Active()
        #   里启动，而本项目为了避开 executor 冲突**主动不调用**它，
        #   所以 PatrolNode 从来没有被 spin 过 —— 它上面的任何订阅回调
        #   都不会执行。本项目实际踩过：订阅建好了，但缓存永远是 None，
        #   于是 AI 一直说「AMCL 可能未收敛」。
        #
        # 这里用一个**自己的节点 + 自己的 spin 线程**，与 PatrolNode 的
        # executor 完全解耦，稳且不产生冲突。
        # ==========================================================
        self._pose_node = None
        self._pose_latest = None        # {'x','y','yaw'}
        self._pose_stamp = None         # time.time()
        self._pose_std = None           # (σ_xy, σ_yaw) 供定位质量判断
        # /localization_status（auto_localizer 发的 JSON）缓存。
        # 里面有激光-地图匹配率、位姿稳定性、进度等，比只看 σ 靠谱得多。
        self._loc_status = None
        self._loc_status_stamp = None
        self._pose_ready = False
        self._pose_lock = threading.Lock()
        try:
            self._start_pose_monitor()
        except Exception as exc:        # noqa: BLE001 - 不该阻断服务创建
            print('[NavigationService] ⚠️ 启动 /amcl_pose 监听失败: %s' % exc)

    def _start_pose_monitor(self):
        """创建独立节点订阅 /amcl_pose 与 /localization_status，后台线程 spin。"""
        self._ensure_rclpy()

        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String

        class _PoseMonitor(Node):
            def __init__(monitor_self, svc):
                super().__init__('amcl_pose_monitor')
                monitor_self._svc = svc
                monitor_self.sub = monitor_self.create_subscription(
                    PoseWithCovarianceStamped, '/amcl_pose',
                    monitor_self._cb, 10)
                # 定位质量状态：auto_localizer 每 0.5s 发一次 JSON。
                # ★ 用 RELIABLE（默认 QoS）：发布端也是默认值，匹配。
                monitor_self.loc_sub = monitor_self.create_subscription(
                    String, '/localization_status',
                    monitor_self._loc_cb, 10)
                monitor_self.get_logger().info(
                    'amcl_pose_monitor 已启动（独立 spin，供位置查询使用）')

            def _cb(monitor_self, msg):
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                # 协方差对角线：0=var(x)，7=var(y)，35=var(yaw)
                # 转成标准差，方便和"米/弧度"直接比。
                cov = msg.pose.covariance
                try:
                    std_xy = math.sqrt(max(0.0, max(cov[0], cov[7])))
                    std_yaw = math.sqrt(max(0.0, cov[35]))
                except Exception:       # noqa: BLE001
                    std_xy = std_yaw = float('inf')
                with monitor_self._svc._pose_lock:
                    monitor_self._svc._pose_latest = {
                        'x': float(p.x), 'y': float(p.y), 'yaw': float(yaw)}
                    monitor_self._svc._pose_stamp = time.time()
                    monitor_self._svc._pose_std = (std_xy, std_yaw)

            def _loc_cb(monitor_self, msg):
                """缓存 /localization_status。解析失败就忽略这一帧。"""
                try:
                    data = json.loads(msg.data)
                except Exception:       # noqa: BLE001
                    return
                if not isinstance(data, dict):
                    return
                with monitor_self._svc._pose_lock:
                    monitor_self._svc._loc_status = data
                    monitor_self._svc._loc_status_stamp = time.time()

        self._pose_node = _PoseMonitor(self)
        self._pose_executor = SingleThreadedExecutor()
        self._pose_executor.add_node(self._pose_node)

        def _spin():
            try:
                self._pose_executor.spin()
            except Exception:           # noqa: BLE001 - 线程退出即可
                pass

        self._pose_thread = threading.Thread(
            target=_spin, daemon=True, name='amcl-pose-spin')
        self._pose_thread.start()
        self._pose_ready = True

    # ----------------------------------------------------------
    # 内部方法
    # ----------------------------------------------------------

    def _ensure_rclpy(self):
        """确保 rclpy 已初始化（线程安全）"""
        if not rclpy.ok():
            rclpy.init()
            self._rclpy_initialized_here = True

    def _ensure_patrol_node(self):
        """惰性初始化 PatrolNode（必须在锁内调用）"""
        if self._patrol_node is None:
            from autopatrol_robot.patrol_node import PatrolNode
            self._ensure_rclpy()
            self._patrol_node = PatrolNode()

    def _ensure_nav2_ready(self, timeout: float = 30.0) -> bool:
        """确保 Nav2 就绪（首次调用时短暂等待 action servers，之后直接返回）。

        ★ 这里**绝不发布 /initialpose**。
        原实现在此处硬发一个 (initial_point_) 的初始位姿，也就是地图原点，
        与 `auto_localizer` 的自动全局定位直接冲突：
          auto_localizer 把粒子云撒满全图 → 本函数又把先验钉到原点
          → 谁后生效谁说了算，结果随机，且大概率被拉回原点。
        实测现象就是 /amcl_pose 一直报 (0.00, 0.00, 0.00)。
        定位初始化统一交给 auto_localizer；只在用户显式要求时
        （initialize_pose 工具）才人工设一个。
        """
        if self._nav2_ready:
            return True

        # 没有 PatrolNode 就没有 action client，无从等待
        if self._patrol_node is None:
            return False

        self._emit_status("nav2_waiting", "等待导航系统就绪...")
        try:
            # Nav2 launch 已经把所有节点起来了，action server 通常 1~2s 内上线。
            # 不调用 waitUntilNav2Active：它内部的 spin_until_future_complete
            # 会与 BasicNavigator 的后台 spin 线程冲突。
            time.sleep(1.0)

            self._nav2_ready = True
            self._emit_status("nav2_ready", "导航系统就绪")
            # NavigationService 不是 rclpy Node，没有 get_logger()，用 print
            print('[NavigationService] Nav2 已就绪'
                  '（定位由 auto_localizer 自动完成，本节点不改动 /initialpose）')
            return True
        except Exception as e:
            self._emit_status("nav2_error", f"导航系统就绪失败: {e}")
            return False

    def _emit_status(self, event_type: str, message: str):
        """通过回调推送状态"""
        if self._status_callback:
            try:
                self._status_callback(event_type, message)
            except Exception:
                pass

    def _set_state(self, state: NavState):
        """设置状态并推送（必须在锁内或可重入场景调用）"""
        self._state = state
        self._emit_status("nav_state", state.value)

    # ----------------------------------------------------------
    # Public API — 导航控制
    # ----------------------------------------------------------

    def wait_until_ready(self, timeout: float = 30.0) -> bool:
        """
        等待 Nav2 导航系统完全就绪。
        首次调用会阻塞直到 Nav2 可用，后续调用立即返回。

        Args:
            timeout: 超时秒数

        Returns:
            True 表示就绪，False 表示超时或失败
        """
        with self._lock:
            self._ensure_patrol_node()
            return self._ensure_nav2_ready(timeout)

    def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> str:
        """
        导航到指定坐标点。

        这是核心导航方法。调用时会：
          1. 等待 Nav2 就绪（首次）
          2. 启动导航任务
          3. 阻塞等待到达或取消

        可以在连续调用中反复使用 —— 不会重复初始化。

        Args:
            x: 目标 X 坐标（米）
            y: 目标 Y 坐标（米）
            yaw: 目标朝向（弧度）

        Returns:
            结果描述字符串
        """
        # ---- 准备阶段（在锁内） ----
        with self._lock:
            if self._state == NavState.NAVIGATING:
                return "⚠️ 已有导航任务进行中，请先取消"

            # 检查 CLI 导航是否正在进行（与 nav_manager 互斥）
            nav_lock = get_global_nav_lock()
            if not nav_lock.acquire(blocking=False):
                return "⚠️ CLI 导航任务进行中，请稍后再试"
            nav_lock.release()

            self._ensure_patrol_node()
            if not self._ensure_nav2_ready():
                return "❌ 导航系统未就绪"

            self._cancel_requested = False
            self._set_state(NavState.NAVIGATING)
            self._patrol_node.publish_nav_status(True)

        # ---- 执行导航（在锁外，避免阻塞其他操作） ----
        try:
            target_pose = self._patrol_node.get_pose_by_xyyaw(x, y, yaw)
            self._emit_status("navigation_start", f"正在前往 ({x:.2f}, {y:.2f})")

            # 使用 goToPose（非阻塞）+ 轮询方式
            self._patrol_node.goToPose(target_pose)

            # ============================================================
            # 零剩余时间看门狗检测到达（不调用 isTaskComplete / getResult）
            #
            # isTaskComplete() 内部 rclpy.spin_until_future_complete 会
            # 创建临时 SingleThreadedExecutor 并添加 PatrolNode，与
            # BasicNavigator 自身的 MultiThreadedExecutor 产生「双重执行器」
            # 冲突，导致 rclpy 内部 guard condition 损坏、导航系统崩溃。
            # getResult() 依赖 isTaskComplete 设置的 status 字段，
            # 不调用 isTaskComplete 则 getResult 永远返回 UNKNOWN。
            #
            # 因此完全依赖看门狗检测到达，到达后直接标记 COMPLETED，
            # 不调用 getResult/cancelTask（后者也 spin）。
            # ============================================================
            zero_remaining_count = 0
            ZERO_REMAINING_THRESHOLD = 15  # 连续 15 次（~3s）→ 视为到达

            while True:
                # 检查取消请求
                with self._lock:
                    if self._cancel_requested:
                        # cancelTask() 也 spin，直接标记 CANCELED 不调用
                        self._set_state(NavState.CANCELED)
                        self._patrol_node.publish_nav_status(False)
                        return "🛑 导航已取消"

                # 获取并推送反馈
                feedback = self._patrol_node.getFeedback()
                if feedback:
                    remaining = Duration.from_msg(
                        feedback.estimated_time_remaining
                    ).nanoseconds / 1e9
                    self._emit_status("navigation_feedback",
                                      f"预计 {remaining:.0f} 秒后到达")

                    # 看门狗：连续 N 次剩余 ≤0 秒 → 视为已到达
                    if remaining <= 0.0:
                        zero_remaining_count += 1
                        if zero_remaining_count >= ZERO_REMAINING_THRESHOLD:
                            self._emit_status("navigation_feedback",
                                              f"已到达（反馈持续 {ZERO_REMAINING_THRESHOLD} 次报 0s，强制结束）")
                            break
                    else:
                        zero_remaining_count = 0

                time.sleep(0.2)

            # ---- 看门狗触发 = 导航完成 ----
            # 不调用 isTaskComplete/getResult/cancelTask（全部有 spin 冲突）
            with self._lock:
                self._set_state(NavState.COMPLETED)
                self._patrol_node.publish_nav_status(False)
                msg = f"✅ 已到达目标点 ({x:.2f}, {y:.2f})"

                # ===== 防御：记录实际到达位姿 =====
                try:
                    actual_pose = self._patrol_node.get_current_pose(timeout_sec=3.0)
                    if actual_pose:
                        _, _, yaw = euler_from_quaternion([
                            actual_pose.rotation.x,
                            actual_pose.rotation.y,
                            actual_pose.rotation.z,
                            actual_pose.rotation.w,
                        ])
                        self._last_arrival_pose = (
                            actual_pose.translation.x,
                            actual_pose.translation.y,
                            yaw,
                        )
                except Exception:
                    pass
                # ===== 防御结束 =====

            # ---- 到达后触发舵机播报（锁外，不阻塞其他操作） ----
            try:
                self._trigger_servo()
            except Exception as e:
                self._emit_status("servo_error", f"舵机播报失败: {e}")
            return msg

        except Exception as e:
            with self._lock:
                self._set_state(NavState.FAILED)
                self._patrol_node.publish_nav_status(False)
            return f"❌ 导航异常: {str(e)}"

    def cancel(self) -> str:
        """
        取消当前导航任务。

        Returns:
            结果描述
        """
        with self._lock:
            if self._state != NavState.NAVIGATING:
                return "ℹ️ 没有进行中的导航任务"

            self._cancel_requested = True
            # 实际的 cancelTask 在导航循环中被调用
            self._set_state(NavState.CANCELED)

        self._emit_status("navigation_canceled", "导航已取消")
        return "✅ 已请求取消导航"

    def get_current_position(self, detailed: bool = False):
        """
        获取机器人当前位置（纯查询，完全不修改 AMCL 状态）。

        实现顺序：
          1. PatrolNode 内缓存的 /amcl_pose（微秒级，首选）
          2. TF map→base_link（缓存没有数据时回退）

        ★ 不再用 `ros2 topic echo --once` 子进程方案：
          - 每次调用要 0.5~5s（进程启动 + 等第一帧），AI 连问几次就卡住
          - 依赖子进程的 ROS 环境，web 服务从异常环境启动时会失效

        ★ 不再要求 _nav2_ready 为真：
          这是一个纯查询接口，和 Nav2 的 action server 是否就绪无关。
          原实现 `if not self._nav2_ready: return None` 导致首次问「我在哪」
          必定失败（因为 _nav2_ready 只有走过一次导航/初始化才会变 True），
          AI 于是误报「AMCL 可能未收敛」——本项目实际踩过。

        Args:
            detailed: True 时返回 (pos, reason) 便于上层给出精确提示

        Returns:
            detailed=False: {'x','y','yaw'} 或 None
            detailed=True : ({'x','y','yaw'} 或 None, 说明字符串)
        """
        reason = ''
        pos = None

        try:
            # ---- 方式1：独立订阅节点缓存的最新 /amcl_pose ----
            # 不再依赖 PatrolNode（它没有被 spin，回调永远不执行）
            cached = None
            age = None
            with self._pose_lock:
                if self._pose_latest is not None and self._pose_stamp:
                    cached = dict(self._pose_latest)
                    age = time.time() - self._pose_stamp

            if cached is not None and age is not None and age <= 3.0:
                pos = cached
                reason = 'AMCL 位姿（%.1fs 前）' % age
            elif cached is not None and age is not None:
                reason = 'AMCL 最近一次位姿已过期（%.1fs 前）' % age
            else:
                reason = '尚未收到任何 /amcl_pose 消息'

            # ---- 方式2：回退到 TF（需要 PatrolNode，仅作兜底） ----
            if pos is None:
                try:
                    with self._lock:
                        self._ensure_patrol_node()
                        node = self._patrol_node
                    transform = node.get_current_pose()
                    if transform is not None:
                        _, _, yaw = euler_from_quaternion([
                            transform.rotation.x,
                            transform.rotation.y,
                            transform.rotation.z,
                            transform.rotation.w
                        ])
                        pos = {
                            'x': float(transform.translation.x),
                            'y': float(transform.translation.y),
                            'yaw': float(yaw),
                        }
                        reason = 'TF map→base_link'
                except Exception:      # noqa: BLE001 - TF 兜底失败不影响主结论
                    pass
        except Exception as exc:              # noqa: BLE001
            reason = '查询过程异常: %s' % exc

        if detailed:
            return pos, reason
        return pos

    def get_localization_quality(self):
        """返回当前 AMCL 定位质量：(ok, std_xy, std_yaw, reason)。

        ============================================================
        为什么需要这个接口（2026-11-01 实机事故）
        ============================================================
        现场：载入地图后 auto_localizer 做了全局定位（粒子云撒满全图），
        120 秒都没收敛（σ_xy 一直在 0.1~1.7m 之间晃）。可这时候用户说了
        「去厕所」，导航照样启动 —— 车按着**错误的位姿**往前开，位姿在
        地图上乱飘，越开 σ 越大，120s 到期又重撒一遍粒子，形成
        「有规律乱飘」的死循环。

        结论：**定位没收敛就不该允许导航**。这个判断必须放在代码层，
        不能指望用户或 AI 自己去注意日志。

        ============================================================
        2026-11-01 二次修订：优先用 /localization_status
        ============================================================
        只看 σ 会两头出错：
          * 真收敛了但 σ 因为粒子略散而超线 → 误挡（事故里 σ=0.112m）
          * 假收敛了但 σ 很小               → 误放
        现在优先读 auto_localizer 发的 /localization_status，里面有
        激光-地图匹配率这个物理证据，以及 auto_localizer 自己的综合
        判定（ready / reason / hint）。

        状态话题不可用（auto_localizer 没在跑）时，退回只看 σ。
        """
        ok_xy = float(os.environ.get('FISHBOT_LOC_OK_STD_XY', '0.30'))
        ok_yaw = float(os.environ.get('FISHBOT_LOC_OK_STD_YAW', '0.30'))

        with self._pose_lock:
            stamp = self._pose_stamp
            std = self._pose_std
            status = dict(self._loc_status) if self._loc_status else None
            status_stamp = self._loc_status_stamp

        # ---------- 路线 1：有 /localization_status ----------
        if status is not None and status_stamp is not None:
            if time.time() - status_stamp <= 5.0:   # 发布周期 0.5s
                ready = bool(status.get('ready'))
                std_xy = status.get('std_xy')
                std_xy = None if std_xy is None else float(std_xy)
                std_yaw_deg = status.get('std_yaw_deg')
                std_yaw = (None if std_yaw_deg is None
                           else float(std_yaw_deg) / 57.29578)
                reason = str(status.get('reason') or '')
                if ready:
                    extra = []
                    mr = status.get('match_rate')
                    if mr is not None:
                        extra.append('激光匹配率 %.0f%%' % (float(mr) * 100))
                    if std_xy is not None:
                        extra.append('σ_xy %.3fm' % std_xy)
                    return (True, std_xy, std_yaw,
                            '定位已就绪'
                            + ('（%s）' % '，'.join(extra) if extra else ''))

                # 未就绪：把原因说清楚，别只甩一句"未收敛"
                parts = []
                if reason:
                    parts.append(reason)
                mr = status.get('match_rate')
                if mr is not None:
                    parts.append('激光匹配率 %.0f%%' % (float(mr) * 100))
                if std_xy is not None:
                    parts.append('σ_xy %.3fm' % std_xy)
                hint = str(status.get('hint') or '')
                msg = '定位未就绪' + ('：' + '，'.join(parts) if parts else '')
                if hint:
                    msg += '\n   ' + hint
                return False, std_xy, std_yaw, msg
            # 话题过期：auto_localizer 可能已经停了，继续走 σ 路线

        # ---------- 路线 2：没有状态话题，只能看 σ ----------
        if stamp is None or std is None:
            # 从没收到过 /amcl_pose：可能是 AMCL 刚重启。
            # 此时无法判断，放行（否则每次冷启动都导航不了）。
            return True, None, None, '尚未收到 /amcl_pose，无法判断定位质量'

        age = time.time() - stamp
        if age > 3.0:
            return True, std[0], std[1], 'AMCL 位姿已过期（%.1fs 前）' % age

        std_xy, std_yaw = std
        if std_xy <= ok_xy and std_yaw <= ok_yaw:
            return (True, std_xy, std_yaw,
                    '定位良好（σ_xy=%.2fm σ_yaw=%.1f°）'
                    % (std_xy, std_yaw * 57.29578))
        return (False, std_xy, std_yaw,
                '定位未收敛（σ_xy=%.2fm σ_yaw=%.1f°，门限 %.2fm/%.1f°）'
                % (std_xy, std_yaw * 57.29578,
                   ok_xy, ok_yaw * 57.29578))

    def get_localization_status_raw(self):
        """返回 /localization_status 的原始 dict（供网页状态条使用）。

        没有数据或已过期（>5s）时返回 None。
        """
        with self._pose_lock:
            status = dict(self._loc_status) if self._loc_status else None
            stamp = self._loc_status_stamp
        if status is None or stamp is None:
            return None
        age = time.time() - stamp
        if age > 5.0:
            return None
        status['age'] = round(age, 2)
        return status

    def get_target_points(self) -> list:
        """
        获取预定义的导航目标点列表。

        Returns:
            [(x, y, yaw), ...] 列表
        """
        with self._lock:
            self._ensure_patrol_node()

        try:
            return self._patrol_node.get_target_points()
        except Exception:
            return []

    def get_state(self) -> dict:
        """
        获取当前导航状态（无锁读，快速返回）。

        Returns:
            {'state': str, 'nav2_ready': bool}
        """
        with self._lock:
            return {
                'state': self._state.value,
                'nav2_ready': self._nav2_ready
            }

    def initialize_amcl_pose(self, x: Optional[float] = None, y: Optional[float] = None) -> str:
        """
        初始化 AMCL 定位系统的初始位姿（设置机器人在 map 中的起始位置）。

        用法：
          - 地图加载后、首次导航前调用一次
          - 如果 AMCL 已初始化（_nav2_ready=True），直接返回不重复操作
          - AI Agent / Web UI 均可安全重复调用

        Args:
            x: 初始 X 坐标（米），None 则使用 patrol_config.yaml 中的 initial_point
            y: 初始 Y 坐标（米），None 则使用 patrol_config.yaml 中的 initial_point

        Returns:
            结果描述字符串
        """
        with self._lock:
            if self._nav2_ready:
                return "ℹ️ AMCL 已初始化，无需重复设置"

            self._ensure_patrol_node()

            px = x if x is not None else self._patrol_node.initial_point_[0]
            py = y if y is not None else self._patrol_node.initial_point_[1]

            from geometry_msgs.msg import PoseWithCovarianceStamped
            initial_pose_msg = PoseWithCovarianceStamped()
            initial_pose_msg.header.frame_id = 'map'
            initial_pose_msg.header.stamp = self._patrol_node.get_clock().now().to_msg()
            initial_pose_msg.pose.pose.position.x = px
            initial_pose_msg.pose.pose.position.y = py
            initial_pose_msg.pose.pose.position.z = 0.0
            initial_pose_msg.pose.pose.orientation.w = 1.0
            initial_pose_msg.pose.covariance[0] = 0.25
            initial_pose_msg.pose.covariance[7] = 0.25
            initial_pose_msg.pose.covariance[35] = 0.0685

            pub = self._patrol_node.create_publisher(
                PoseWithCovarianceStamped, '/initialpose', 10)
            pub.publish(initial_pose_msg)
            self._patrol_node.initial_pose_received = True

            self._emit_status("nav2_waiting",
                              f"已发布初始位姿 ({px:.2f}, {py:.2f})，等待 AMCL 收敛...")
            time.sleep(2.0)

            self._patrol_node.destroy_publisher(pub)
            self._nav2_ready = True
            self._emit_status("nav2_ready", "AMCL 初始位姿设置完成")

        return f"✅ AMCL 初始位姿已设置为 ({px:.2f}, {py:.2f})"

    def recover_initial_pose(self) -> Optional[str]:
        """
        使用上一次导航成功到达时的实际位姿恢复 AMCL 初始位姿。

        防御场景：
          - 如果 AMCL 因任何原因被意外 reset（如重复调用 waitUntilNav2Active），
            调用此方法可用上次到达位姿重新设置 /initialpose。

        Returns:
            成功时返回确认消息，无可用位姿时返回 None
        """
        with self._lock:
            if self._last_arrival_pose is None:
                return None
            self._ensure_patrol_node()
            x, y, yaw = self._last_arrival_pose

        try:
            self._patrol_node.setInitialPose(
                self._patrol_node.get_pose_by_xyyaw(x, y, yaw)
            )
            self._emit_status(
                "pose_recovered",
                f"♻️ 已用上次到达位姿恢复 AMCL: ({x:.2f}, {y:.2f})"
            )
            return f"✅ 已用上次到达位姿 ({x:.2f}, {y:.2f}) 恢复 AMCL 初始位姿"
        except Exception as e:
            return f"❌ 恢复初始位姿失败: {e}"

    # ----------------------------------------------------------
    # 舵机触发（内部使用）
    # ----------------------------------------------------------

    def _trigger_servo(self) -> str:
        """到达目标点后先停电机，再异步发送舵机播报（不等待按钮）。

        注意：不调用 call_servo_and_wait()，其 spin_once 与
        BasicNavigator 后台 MultiThreadedExecutor 冲突 PatrolNode。
        改用 call_servo_action() 发送即走，ESP32 自行处理播报→等按钮。

        Returns:
            'sent' / 'servo_unavailable'
        """
        from autopatrol_robot.patrol_node import ServoClient
        servo_client = ServoClient(node=self._patrol_node)
        servo_client.stop_robot()
        time.sleep(0.3)
        ok = servo_client.call_servo_action()
        if ok:
            self._emit_status("servo_sent", "舵机播报指令已发送（ESP32 异步执行）")
            return 'sent'
        else:
            self._emit_status("servo_error", "舵机播报服务不可用")
            return 'servo_unavailable'

    # ----------------------------------------------------------
    # 资源清理
    # ----------------------------------------------------------

    def shutdown(self):
        """
        清理所有 ROS2 资源。

        执行步骤：
          1. 取消进行中的导航任务
          2. 等待导航循环退出
          3. 销毁 PatrolNode（BasicNavigator 的后台 spin 线程随之停止）
          4. 重置全局状态

        应用退出时由 web_controller.py 或 force_cleanup_all() 调用。
        """
        # 步骤 1: 标记取消请求（不调用 cancelTask，它会 spin PatrolNode）
        with self._lock:
            self._cancel_requested = True

        # 步骤 2: 等待导航循环退出（最多 5 秒）
        for _ in range(25):
            with self._lock:
                if self._state != NavState.NAVIGATING:
                    break
            time.sleep(0.2)

        # 步骤 2.5: 停止独立的 /amcl_pose 监听线程
        # 必须先关 executor 再关节点：直接 destroy_node 会让正在 spin 的
        # executor 抛 InvalidHandle，虽然被兜住但会刷警告。
        try:
            if getattr(self, '_pose_executor', None) is not None:
                self._pose_executor.shutdown(timeout_sec=1.0)
        except Exception:               # noqa: BLE001
            pass
        try:
            if getattr(self, '_pose_node', None) is not None:
                self._pose_node.destroy_node()
        except Exception:               # noqa: BLE001
            pass
        self._pose_node = None
        self._pose_executor = None

        # 步骤 3: 销毁 PatrolNode
        # BasicNavigator.destroy_node() 会停止 _spin_thread
        with self._lock:
            if self._patrol_node is not None:
                try:
                    self._patrol_node.publish_nav_status(False)
                except Exception:
                    pass
                try:
                    self._patrol_node.destroy_node()
                except Exception as e:
                    print(f"[NavigationService] 销毁 PatrolNode 异常: {e}")
                self._patrol_node = None
                self._nav2_ready = False

            self._set_state(NavState.IDLE)

            # 注意: 不在这里调用 rclpy.shutdown()，
            # 由 web_controller.py 的 main() 统一调用。


# ============================================================
# 全局单例（方便各模块共享）
# ============================================================
# 在 web_controller.py 中创建并赋值
_global_nav_service: Optional[NavigationService] = None


def get_global_nav_service() -> Optional[NavigationService]:
    """获取全局 NavigationService 单例"""
    return _global_nav_service


def set_global_nav_service(service: NavigationService):
    """设置全局 NavigationService 单例"""
    global _global_nav_service
    _global_nav_service = service
