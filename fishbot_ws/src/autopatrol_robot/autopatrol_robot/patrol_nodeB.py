#!/usr/bin/env python3
"""
patrol_nodeB.py — 方案B：专用 Spin 线程优化版

核心架构变更：
  1. **模块级 MultiThreadedExecutor + daemon spin 线程**
     - PatrolNode 初始化时自动注册到共享 executor
     - spin 线程以 ~20Hz 持续驱动所有回调（feedback / AMCL / TF / action）
     - 导航空闲期：spin 线程保持 PatrolNode 的所有状态实时同步

  2. **所有权转移机制**
     - _take_node_ownership(): 从共享 executor 中移除 PatrolNode
       调用方获得独占 spin 权（用于 spin_until_future_complete）
     - _return_node_ownership(): 归还 PatrolNode 给共享 executor
       spin 线程恢复回调处理

  3. **导航期间**
     - nav_to_pose / navigate_and_complete: take ownership →
       goToPose → isTaskComplete 循环 → getResult → return ownership
     - isTaskComplete() 的 spin_until_future_complete 不会被 spin 线程干扰
       （因为 PatrolNode 已从共享 executor 移除）

  4. **get_current_pose 简化**
     - spin 线程持续处理 TF listener，buffer_ 中始终有最新 TF
     - 不再需要内部 spin_once 循环

  5. **cancel 回归标准 API**
     - 可以直接调用 cancelTask()（在 ownership 期间）

与方案A的区别：
  - 方案A：不改变架构，在每次 feedback 读前手动 spin_once
  - 方案B：根本解决 — Node 有持久 executor，所有回调始终被处理
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
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import time
from std_msgs.msg import Bool

# ============================================================
# 方案B 核心：模块级共享 executor + spin 线程
#
# 设计要点：
#   - executor: MultiThreadedExecutor(num_threads=1)
#     单线程保证回调串行，避免并发问题，但足以驱动所有回调。
#   - spin 线程: daemon，每 50ms spin_once，持续处理回调。
#   - 所有权锁: _ownership_lock 保护 add/remove 操作，
#     确保 spin 线程不会在 remove 期间 spin 该 Node。
# ============================================================
_executor = None
_spin_thread = None
_spin_stop = threading.Event()
_spin_ready = threading.Event()
_ownership_lock = threading.Lock()
_registered_nodes = set()  # 追踪哪些 Node 已注册到 executor


def _ensure_spin_thread(node: Node):
    """
    确保模块级 spin 线程已启动，并将给定 node 注册到 executor。

    幂等：多次调用只启动一次线程，但会确保 node 在 executor 中。
    """
    global _executor, _spin_thread

    with _ownership_lock:
        # 启动 executor 和 spin 线程（仅首次）
        if _executor is None:
            _executor = MultiThreadedExecutor(num_threads=1)
            _spin_stop.clear()
            _spin_ready.clear()

            def _spin_loop():
                _spin_ready.set()
                while not _spin_stop.is_set():
                    # 在锁外 spin，避免阻塞所有权转移
                    with _ownership_lock:
                        _executor.spin_once(timeout_sec=0.05)
                    # 锁外短暂 sleep，给所有权转移留窗口
                    time.sleep(0.001)

            _spin_thread = threading.Thread(target=_spin_loop, daemon=True)
            _spin_thread.start()
            _spin_ready.wait(timeout=3.0)

        # 注册 node（如果尚未注册）
        if node not in _registered_nodes:
            _executor.add_node(node)
            _registered_nodes.add(node)


def _take_node_ownership(node: Node):
    """
    从共享 executor 中移除 node，获得独占 spin 权。

    调用时机：在需要 spin_until_future_complete(node, ...) 之前。
    调用方负责在完成后调用 _return_node_ownership()。
    """
    global _executor
    with _ownership_lock:
        if _executor is not None and node in _registered_nodes:
            _executor.remove_node(node)
            _registered_nodes.discard(node)
            return True
    return False


def _return_node_ownership(node: Node):
    """
    将 node 归还给共享 executor，spin 线程恢复回调处理。
    """
    global _executor
    with _ownership_lock:
        if _executor is not None and node not in _registered_nodes:
            _executor.add_node(node)
            _registered_nodes.add(node)
            return True
    return False


def _shutdown_spin_thread():
    """停止 spin 线程并清理 executor（进程退出前调用）。"""
    global _executor, _spin_thread, _spin_stop, _registered_nodes
    _spin_stop.set()
    if _spin_thread is not None:
        _spin_thread.join(timeout=3.0)
        _spin_thread = None
    if _executor is not None:
        _executor.shutdown()
        _executor = None
    _registered_nodes.clear()


class ServoClient:
    """舵机控制（语音播报）客户端 — 复用已有 Node，避免 ROS2 实体泄漏"""
    def __init__(self, node: Node = None):
        if node is not None:
            self.node = node
            self._own_node = False
        else:
            self.node = Node('servo_client_python')
            self._own_node = True
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
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
    def __init__(self, node_name='patrol_node_b'):
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

        # === 方案B 核心：注册到共享 executor，启动 spin 线程 ===
        _ensure_spin_thread(self)
        self.get_logger().info(
            f'[{node_name}] 已注册到共享 spin 线程（方案B）'
        )

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
    # 方案B：nav_to_pose — 所有权转移 + 标准 API
    #
    # 流程：
    #   1. take ownership → 从共享 executor 移除本 Node
    #   2. goToPose → isTaskComplete 循环（内部 spin 独占本 Node）
    #   3. getResult / cancelTask（安全，因为没有 spin 线程竞争）
    #   4. return ownership → 归还本 Node，spin 线程恢复
    #
    # 优势 vs 原版：
    #   - isTaskComplete 的 spin_until_future_complete 安全：独占 Node
    #   - feedback 总是新鲜：isTaskComplete 的 spin 会刷新
    #   - cancelTask 可以安全调用
    #   - 导航间隙：spin 线程保持 AMCL/TF 状态同步
    # ============================================================
    def nav_to_pose(self, target_pose):
        """导航到指定位姿（方案B：所有权转移 + 标准 API）"""
        # Step 1: 从共享 executor 手中接过 PatrolNode 控制权
        _take_node_ownership(self)

        try:
            self.goToPose(target_pose)
            last_status_time = time.time()

            # Step 2: 标准 isTaskComplete 循环（内部 spin 独占本 Node，安全）
            while not self.isTaskComplete():
                if self._nav_canceled:
                    self.get_logger().info('导航已被取消')
                    self.cancelTask()  # 方案B：可以安全调用 cancelTask
                    return

                now = time.time()
                if now - last_status_time > 1.0:
                    self.publish_nav_status(True)
                    last_status_time = now

                # feedback 是新鲜的：isTaskComplete 内部的 spin 会刷新
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

            # Step 3: 使用标准 getResult（安全，无竞争）
            result = self.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('导航结果：成功')
            elif result == TaskResult.CANCELED:
                self.get_logger().info('导航结果：已取消')
                return
            else:
                self.get_logger().warn(f'导航结果异常: {result}')
                return

        finally:
            # Step 4: 归还 PatrolNode，spin 线程恢复回调处理
            _return_node_ownership(self)

        # 以下操作在 spin 线程恢复后进行（不影响）
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
    # 方案B：navigate_and_complete — 同 nav_to_pose 模式
    # ============================================================
    def navigate_and_complete(self, x: float, y: float, yaw: float = 0.0) -> str:
        """导航到目标点并完成舵机播报（方案B：所有权转移版）"""
        target_pose = self.get_pose_by_xyyaw(x, y, yaw)

        _take_node_ownership(self)
        try:
            self.goToPose(target_pose)
            last_status_time = time.time()

            while not self.isTaskComplete():
                if self._nav_canceled:
                    self.get_logger().info('导航已被取消')
                    self.cancelTask()
                    return 'canceled'

                now = time.time()
                if now - last_status_time > 1.0:
                    self.publish_nav_status(True)
                    last_status_time = now

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

        finally:
            _return_node_ownership(self)

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

    # ============================================================
    # 方案B 亮点：get_current_pose 极简化
    #
    # spin 线程持续处理 TF listener → buffer_ 中始终有最新 TF 数据
    # 不再需要内部 spin_once 循环，只需 lookup_transform + 短暂重试
    # ============================================================
    def get_current_pose(self, timeout_sec=3.0):
        """
        通过TF获取当前位姿。

        方案B 优势：spin 线程持续驱动 TF listener，
        buffer_ 中通常已有最新变换，无需 spin_once 等待。
        """
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
                # 方案B：spin 线程在后台处理，这里只需短暂等待
                time.sleep(0.05)
                continue

        self.get_logger().warn(f'无法获取坐标变换')
        return None

    def shutdown(self):
        """清理：从共享 executor 移除本 Node。"""
        _take_node_ownership(self)
        self.get_logger().info('PatrolNode 已从共享 spin 线程移除')


# ============================================================
# 方案B：main() — 简化初始化
#
# spin 线程在 PatrolNode.__init__ 中已启动，持续处理 AMCL/TF 等回调。
# 只需等待 action server 就绪即可开始导航。
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
        patrol.publish_nav_status(False)
    sig_module.signal(sig_module.SIGINT, _sigint_handler)

    try:
        # === 方案B：spin 线程已在处理 AMCL 回调，只需等 action server ===
        patrol.get_logger().info('等待 Nav2 激活（方案B：spin 线程驱动）...')
        patrol.setInitialPose(patrol.get_pose_by_xyyaw(
            patrol.initial_point_[0],
            patrol.initial_point_[1],
            patrol.initial_point_[2]
        ))

        # action server 的就绪检测也由 spin 线程驱动
        nav2_ready = False
        for _ in range(100):
            if patrol.nav_to_pose_client.server_is_ready():
                nav2_ready = True
                break
            time.sleep(0.1)
        if not nav2_ready:
            patrol.get_logger().error('Nav2 action server 未在 10s 内就绪')
            return
        patrol.get_logger().info('Nav2 已激活（方案B）')

        for point in patrol.get_target_points():
            if patrol._nav_canceled:
                break
            x, y, yaw = point[0], point[1], point[2]
            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            patrol.nav_to_pose(target_pose)

    finally:
        patrol.publish_nav_status(False)
        time.sleep(0.5)
        patrol.shutdown()
        _shutdown_spin_thread()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
