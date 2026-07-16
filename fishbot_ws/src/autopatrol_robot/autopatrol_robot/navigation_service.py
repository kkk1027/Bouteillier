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
        """确保 Nav2 已就绪（首次调用时阻塞等待，之后直接返回）"""
        if self._nav2_ready:
            return True

        # 只有第一次需要等待
        if self._patrol_node is None:
            return False

        self._emit_status("nav2_waiting", "等待导航系统就绪...")
        try:
            # 1. 用 ROS2 原生 publisher 发布初始位姿（绕过 BasicNavigator.setInitialPose）
            from geometry_msgs.msg import PoseWithCovarianceStamped

            initial_pose_msg = PoseWithCovarianceStamped()
            initial_pose_msg.header.frame_id = 'map'
            initial_pose_msg.header.stamp = self._patrol_node.get_clock().now().to_msg()
            initial_pose_msg.pose.pose.position.x = self._patrol_node.initial_point_[0]
            initial_pose_msg.pose.pose.position.y = self._patrol_node.initial_point_[1]
            initial_pose_msg.pose.pose.position.z = 0.0
            initial_pose_msg.pose.pose.orientation.w = 1.0
            initial_pose_msg.pose.covariance[0] = 0.25
            initial_pose_msg.pose.covariance[7] = 0.25
            initial_pose_msg.pose.covariance[35] = 0.0685

            pub = self._patrol_node.create_publisher(
                PoseWithCovarianceStamped, '/initialpose', 10)
            pub.publish(initial_pose_msg)
            self._emit_status("nav2_waiting", "已发布初始位姿，等待 AMCL 收敛...")

            # 2. 不调用 spin_once — BasicNavigator 后台 executor 会自动刷新发布队列
            #    initialpose 消息会在 sleep 2s 期间被发送到 AMCL
            self._patrol_node.initial_pose_received = True

            # 4. 等待 Nav2 action servers 就绪 + AMCL 收敛
            #    不调用 waitUntilNav2Active（内部 spin_until_future_complete
            #    与 BasicNavigator 后台 spin 线程可能冲突）。
            #    Nav2 launch 已启动所有节点，2s 通常足够 action servers 上线。
            self._emit_status("nav2_waiting", "等待 Nav2 action servers 就绪...")
            time.sleep(2.0)

            # 清理临时 publisher（可选，节点销毁时会自动清理）
            self._patrol_node.destroy_publisher(pub)

            self._nav2_ready = True
            self._emit_status("nav2_ready", "导航系统就绪")
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

    def get_current_position(self) -> Optional[dict]:
        """
        获取机器人当前位置（纯查询，不修改 AMCL 状态）。

        优先通过 ros2 topic echo 直接从 /amcl_pose 获取 AMCL 位姿估计
        （绕过 TF，避免 BasicNavigator 后台 executor 的 TF buffer 填充问题）。
        如果 CLI 方式失败则回退到 TF 方式。

        Returns:
            {'x': float, 'y': float, 'yaw': float} 或 None
        """
        with self._lock:
            self._ensure_patrol_node()
            if not self._nav2_ready:
                return None

        # === 方式1: ros2 topic echo 直接从 /amcl_pose 获取位姿（绕过 TF） ===
        try:
            import subprocess
            import os
            import math

            result = subprocess.run(
                ['ros2', 'topic', 'echo', '--once', '/amcl_pose', '--field', 'pose.pose'],
                capture_output=True, text=True, timeout=5.0,
                env={**os.environ}
            )

            if result.returncode == 0 and result.stdout.strip():
                # 输出格式:
                #   position:
                #     x: 0.001
                #     y: 0.002
                #     z: 0.0
                #   orientation:
                #     x: 0.0
                #     y: 0.0
                #     z: -0.004
                #     w: 0.999
                text = result.stdout

                # 解析 position (第一组 x, y)
                import re
                px_match = re.search(r'^\s*x:\s*([-\d.e+]+)', text, re.MULTILINE)
                py_match = re.search(r'^\s*y:\s*([-\d.e+]+)', text, re.MULTILINE)

                if px_match and py_match:
                    x = float(px_match.group(1))
                    y = float(py_match.group(1))

                    # 解析 orientation (第二组 z, w)
                    all_z = re.findall(r'^\s*z:\s*([-\d.e+]+)', text, re.MULTILINE)
                    all_w = re.findall(r'^\s*w:\s*([-\d.e+]+)', text, re.MULTILINE)

                    oz = float(all_z[-1]) if all_z else 0.0
                    ow = float(all_w[-1]) if all_w else 1.0

                    # 从四元数 (0, 0, oz, ow) 计算 yaw
                    # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
                    siny_cosp = 2.0 * (ow * oz + 0.0 * 0.0)
                    cosy_cosp = 1.0 - 2.0 * (0.0 * 0.0 + oz * oz)
                    yaw = math.atan2(siny_cosp, cosy_cosp)

                    return {'x': x, 'y': y, 'yaw': yaw}
        except Exception:
            pass  # CLI 方式失败，回退到 TF

        # === 方式2: 回退到 TF 方式 ===
        try:
            transform = self._patrol_node.get_current_pose()
            if transform is None:
                return None

            _, _, yaw = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            ])

            return {
                'x': transform.translation.x,
                'y': transform.translation.y,
                'yaw': yaw
            }
        except Exception:
            return None

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
