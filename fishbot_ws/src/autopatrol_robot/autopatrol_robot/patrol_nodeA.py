#!/usr/bin/env python3
"""
patrol_nodeA.py — 方案A：最小外科手术优化版

相对于原 patrol_node.py 的三处精准修复：

  1. **_safe_spin_once()** — 模块级 spin 锁 + spin_once(0.05s)，
     在 nav_to_pose/navigate_and_complete 的每次 feedback 读取前调用，
     确保 feedback 确实被 executor 驱动刷新（而非读冻结的旧值）。

  2. **初始化** — main() 中用 spin_once 轮询替代 waitUntilNav2Active()，
     避免后者在 AMCL 未就绪时创建大量临时 executor。

  3. **取消路径** — 增加 _cancel_nav_goal() 使用 cancel_goal_async 作为备选，
     减少对 cancelTask()（内部有 spin_until_future_complete）的依赖。

设计原则：不改变架构，只精准修复已知缺陷，改动量最小。
"""

import rclpy
import signal
import threading
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.duration import Duration
from std_srvs.srv import SetBool
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import time
from std_msgs.msg import Bool

# ============================================================
# 方案A 新增：模块级 spin 锁
# 确保同一时刻只有一个线程在 spin PatrolNode，防止 rclpy 检测到
# 同一个 Node 被多个 executor 同时添加而抛异常。
# ============================================================
_spin_lock = threading.Lock()


class ServoClient:
    """舵机控制（语音播报）客户端 — 复用已有 Node，避免 ROS2 实体泄漏"""
    def __init__(self, node: Node = None):
        """
        Args:
            node: 可复用的 ROS2 Node。若为 None，则创建独立节点（仅向后兼容）。
        """
        if node is not None:
            self.node = node
            self._own_node = False
        else:
            self.node = Node('servo_client_python')
            self._own_node = True
        # 创建/cmd_vel发布者，用于stop_robot
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        # 创建/imu_corrected发布者，用于在舵机阻塞前注入零角速度
        self.imu_pub = self.node.create_publisher(Imu, '/imu_corrected', 10)
        self._logger = self.node.get_logger()

    def stop_robot(self):
        """发送停止机器人指令，持续刹停直到机器人真正静止。"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0

        for _ in range(30):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.05)

        zero_imu = Imu()
        zero_imu.header.stamp = self.node.get_clock().now().to_msg()
        zero_imu.header.frame_id = 'imu_link'
        self.imu_pub.publish(zero_imu)
        self._logger.info('已发送停止指令 + 零角速度IMU（防止EKF偏航漂移）')

    def call_servo_action(self):
        """调用 ESP32 舵机播报服务（发送即走，不等待按钮响应）"""
        from rclpy.callback_groups import ReentrantCallbackGroup
        client = self.node.create_client(
            SetBool,
            'esp32_command',
            callback_group=ReentrantCallbackGroup()
        )

        start = time.time()
        while not client.service_is_ready():
            if time.time() - start > 5.0:
                self._logger.error('播报服务不可用')
                return False
            time.sleep(0.1)

        req = SetBool.Request()
        req.data = False
        future = client.call_async(req)

        def _done_callback(fut):
            try:
                if fut.result().success:
                    self._logger.info('播报任务完成')
                else:
                    self._logger.warning('播报任务失败（ESP32 返回失败）')
            except Exception as e:
                self._logger.error(f'播报任务异常: {e}')
        future.add_done_callback(_done_callback)
        self._logger.info('已发送舵机播报指令（异步，等待 ESP32 执行）')
        return True

    def call_servo_and_wait(self, timeout: float = 60.0) -> str:
        """同步调用舵机服务，阻塞等待 ESP32 完成「播报→等按钮→返回」全流程。"""
        from rclpy.callback_groups import ReentrantCallbackGroup

        call_node = Node('servo_call_temp')
        try:
            client = call_node.create_client(
                SetBool,
                'esp32_command',
                callback_group=ReentrantCallbackGroup()
            )

            start = time.time()
            while not client.service_is_ready():
                if time.time() - start > 5.0:
                    self._logger.error('播报服务不可用（ESP32 离线？）')
                    return 'servo_unavailable'
                time.sleep(0.1)

            req = SetBool.Request()
            req.data = False

            self._logger.info('📢 发送舵机播报指令，等待 ESP32 按钮确认...')
            future = client.call_async(req)

            rclpy.spin_until_future_complete(call_node, future, timeout_sec=timeout)

            if not future.done():
                self._logger.warning('⏰ 等待 ESP32 按钮按下超时')
                return 'timeout'

            if future.result().success:
                self._logger.info('✅ 舵机播报完成（按钮已按下）')
                return 'success'
            else:
                self._logger.warning('⚠️ 舵机播报失败（ESP32 返回失败）')
                return 'timeout'
        except Exception as e:
            self._logger.error(f'舵机服务异常: {e}')
            return 'error'
        finally:
            call_node.destroy_node()


class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node_a'):
        super().__init__(node_name)
        # 导航相关定义
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, -1.0, 4.7, 3.14, -1.0, -2.5, 0.0, -4.5, -0.55, -1.57])
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        # 实时位置获取 TF 相关定义
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        # 状态机status发布者
        self.nav_status_pub = self.create_publisher(Bool, '/nav_node_status', 10)
        self._nav_canceled = False
        self.publish_nav_status(True)
        time.sleep(0.5)

    # ============================================================
    # 方案A 新增：安全 spin_once
    #
    # 问题：isTaskComplete() 内部的 spin_until_future_complete 会刷新
    # feedback callback，但在边缘情况下（如 goal 刚被 accept 但 feedback
    # 尚未到达），getFeedback() 可能读到旧值或 None。
    #
    # 修复：每次读 feedback 前，用模块级锁保护地 spin_once(0.05s)，
    # 确保至少一帧 callback 被处理。
    # ============================================================
    def _safe_spin_once(self, timeout_sec: float = 0.05):
        """模块级锁保护的 spin_once，防止多线程并发 spin 同一 Node。"""
        with _spin_lock:
            rclpy.spin_once(self, timeout_sec=timeout_sec)

    # ============================================================
    # 方案A 新增：cancel_goal_async 备选路径
    #
    # BasicNavigator.cancelTask() 内部调用 spin_until_future_complete，
    # 在 SIGINT 上下文中可能和 isTaskComplete 的 spin 冲突。
    # cancel_goal_async() 只提交取消请求（返回 future），不 spin 等待，
    # 下一次 isTaskComplete 或 _safe_spin_once 的 spin 会处理该 future。
    # ============================================================
    def _cancel_nav_goal(self):
        """通过 cancel_goal_async 取消当前导航目标（无需 spin 等待）。"""
        if hasattr(self, 'goal_handle') and self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
                self.get_logger().info('已提交导航取消请求（cancel_goal_async）')
                return True
            except Exception as e:
                self.get_logger().warn(f'cancel_goal_async 失败: {e}')
        return False

    def publish_nav_status(self, status):
        """发布导航节点运行状态"""
        msg = Bool()
        msg.data = status
        self.nav_status_pub.publish(msg)
        self.get_logger().info(f'发布导航节点状态: {status}')

    def get_pose_by_xyyaw(self, x, y, yaw):
        """通过 x,y,yaw 合成 PoseStamped"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        rotation_quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = rotation_quat[0]
        pose.pose.orientation.y = rotation_quat[1]
        pose.pose.orientation.z = rotation_quat[2]
        pose.pose.orientation.w = rotation_quat[3]
        return pose

    def get_target_points(self):
        """通过参数值获取目标点集合"""
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_) / 3)):
            x = self.target_points_[index * 3]
            y = self.target_points_[index * 3 + 1]
            yaw = self.target_points_[index * 3 + 2]
            points.append([x, y, yaw])
            self.get_logger().info(f'获取到目标点: {index}->({x},{y},{yaw})')
        return points

    # ============================================================
    # 方案A 修改：nav_to_pose
    #
    # 改动：在 getFeedback() 前调用 _safe_spin_once()，确保 feedback
    # 被 executor 驱动刷新。其余逻辑与原版一致。
    # ============================================================
    def nav_to_pose(self, target_pose):
        """导航到指定位姿（方案A：每次 feedback 读取前 spin_once 确保新鲜度）"""
        self.goToPose(target_pose)
        last_status_time = time.time()

        while not self.isTaskComplete():
            if self._nav_canceled:
                self.get_logger().info('导航已被取消')
                # 方案A：优先使用 cancel_goal_async（无 spin），
                # 失败时回退到 cancelTask（有 spin）
                if not self._cancel_nav_goal():
                    self.cancelTask()
                return
            # 每 ~1s 刷新 nav_status
            now = time.time()
            if now - last_status_time > 1.0:
                self.publish_nav_status(True)
                last_status_time = now

            # === 方案A 核心改动：spin_once 驱动 feedback 刷新 ===
            self._safe_spin_once(timeout_sec=0.05)
            feedback = self.getFeedback()
            if feedback:
                remaining = Duration.from_msg(
                    feedback.estimated_time_remaining
                ).nanoseconds / 1e9
                self.get_logger().info(
                    f'预计剩余: {remaining:.1f}s, '
                    f'距离: {feedback.distance_remaining:.2f}m'
                )
            # 降频到 ~5Hz
            time.sleep(0.1)

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航结果：成功')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('导航结果：已取消')
            return
        else:
            self.get_logger().warn(f'导航结果异常: {result}')
            return

        servo_client = ServoClient(node=self)
        servo_client.stop_robot()
        self.get_logger().info('执行舵机动作')
        time.sleep(0.3)
        result = servo_client.call_servo_and_wait()
        if result == 'success':
            self.get_logger().info('舵机动作完成')
        else:
            self.get_logger().error(f'舵机动作失败: {result}')

    # ============================================================
    # 方案A 修改：navigate_and_complete
    #
    # 同 nav_to_pose：feedback 读取前加 _safe_spin_once()
    # ============================================================
    def navigate_and_complete(self, x: float, y: float, yaw: float = 0.0) -> str:
        """导航到目标点并完成舵机播报（方案A：反馈刷新优化版）"""
        target_pose = self.get_pose_by_xyyaw(x, y, yaw)
        self.goToPose(target_pose)
        last_status_time = time.time()

        while not self.isTaskComplete():
            if self._nav_canceled:
                self.get_logger().info('导航已被取消')
                self._cancel_nav_goal()
                return 'canceled'
            now = time.time()
            if now - last_status_time > 1.0:
                self.publish_nav_status(True)
                last_status_time = now

            # === 方案A 核心改动 ===
            self._safe_spin_once(timeout_sec=0.05)
            feedback = self.getFeedback()
            if feedback:
                remaining = Duration.from_msg(
                    feedback.estimated_time_remaining
                ).nanoseconds / 1e9
                self.get_logger().info(
                    f'预计剩余: {remaining:.1f}s, '
                    f'距离: {feedback.distance_remaining:.2f}m'
                )
            time.sleep(0.1)

        result = self.getResult()
        if result == TaskResult.CANCELED:
            self.get_logger().info('导航已被取消')
            return 'canceled'
        if result != TaskResult.SUCCEEDED:
            self.get_logger().warn(f'导航异常: {result}')
            return 'servo_failed'

        self.get_logger().info('导航到达，执行舵机播报...')
        servo = ServoClient(node=self)
        servo.stop_robot()
        time.sleep(0.3)
        servo_result = servo.call_servo_and_wait()
        if servo_result == 'success':
            self.get_logger().info('✅ 导航+播报 全部完成')
        else:
            self.get_logger().warn(f'导航到达但播报未完成: {servo_result}')
            return 'servo_failed'
        return 'success'

    def get_current_pose(self, timeout_sec=3.0):
        """通过TF获取当前位姿。"""
        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time) < timeout_sec:
            try:
                tf = self.buffer_.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(seconds=0))
                transform = tf.transform

                _, _, yaw = euler_from_quaternion([
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w
                ])

                self.get_logger().info(
                    f'当前位置: x={transform.translation.x:.3f}, '
                    f'y={transform.translation.y:.3f}, yaw={yaw:.3f} 弧度'
                )
                return transform

            except Exception as e:
                self.get_logger().debug(f'等待TF变换... {str(e)[:50]}')
                self._safe_spin_once(timeout_sec=0.1)
                continue

        self.get_logger().warn(f'无法获取坐标变换')
        return None


# ============================================================
# 方案A 修改：main()
#
# 改动：用 spin_once 轮询替代 waitUntilNav2Active()，
# 避免后者在 AMCL 未就绪时内部 spin_until_future_complete
# 循环创建大量临时 executor 的问题。
# ============================================================
def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    patrol = PatrolNode()

    # SIGINT 信号处理
    import signal as sig_module
    def _sigint_handler(signum, frame):
        patrol._nav_canceled = True
        patrol.get_logger().info('收到 SIGINT，正在取消导航...')
        patrol._cancel_nav_goal()
        patrol.publish_nav_status(False)
    sig_module.signal(sig_module.SIGINT, _sigint_handler)

    try:
        # === 方案A 改动：手动 spin_once 轮询等待 Nav2 激活 ===
        patrol.get_logger().info('等待 Nav2 激活...')
        patrol.setInitialPose(patrol.get_pose_by_xyyaw(
            patrol.initial_point_[0],
            patrol.initial_point_[1],
            patrol.initial_point_[2]
        ))
        # 用 spin_once 驱动回调处理（AMCL 需要收到 initialpose 才能初始化）
        # 同时等待 waitUntilNav2Active 内部依赖的 nav_to_pose action server
        nav2_ready = False
        for _ in range(100):  # 最多等 10s（100 × 0.1s）
            patrol._safe_spin_once(timeout_sec=0.05)
            if patrol.nav_to_pose_client.server_is_ready():
                nav2_ready = True
                break
            time.sleep(0.05)
        if not nav2_ready:
            patrol.get_logger().error('Nav2 action server 未在 10s 内就绪')
            return
        patrol.get_logger().info('Nav2 已激活')

        for point in patrol.get_target_points():
            if patrol._nav_canceled:
                break
            x, y, yaw = point[0], point[1], point[2]
            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            patrol.nav_to_pose(target_pose)
    finally:
        patrol.publish_nav_status(False)
        time.sleep(0.5)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
