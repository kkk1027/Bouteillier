import rclpy
import signal
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
        # （ESP32 阻塞期间 /imu_raw 停发 → imu_bias_remover 停发 → EKF 用
        #  最后一帧 vyaw 做预测，必须确保它为 0，否则 RViz 中机器人缓慢旋转）
        self.imu_pub = self.node.create_publisher(Imu, '/imu_corrected', 10)
        self._logger = self.node.get_logger()

    def stop_robot(self):
        """发送停止机器人指令，持续刹停直到机器人真正静止。

        Nav2 标记"到达"时最后的角速度调整可能还在管道中，
        需要持续发布零速 1.5s 以压住残余指令，确保彻底停稳。
        """
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        
        # 持续刹停 ~1.5s（30 次 × 50ms）
        for _ in range(30):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.05)

        # ====== 注入零角速度 IMU，防止 EKF 预测偏航漂移 ======
        # ESP32 舵机阻塞期间 /imu_raw 停发 → imu_bias_remover 停发
        # → EKF sensor_timeout=0.5s 后进入纯预测模式，用最后一帧 vyaw 外推 yaw
        # 若最后一帧 IMU 仍有残留陀螺偏置（0.005 rad/s），60s 累积漂移 ~17°
        # 此处发布 wz=0 的 IMU 消息，确保 EKF 的 vyaw 状态被驱动为 0
        zero_imu = Imu()
        zero_imu.header.stamp = self.node.get_clock().now().to_msg()
        zero_imu.header.frame_id = 'imu_link'
        # angular_velocity 默认全零（z=0.0），无需显式赋值
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
        
        # 用 service_is_ready() 轮询替代 wait_for_service，
        # 避免后者内部 spin_until_future_complete 与 BasicNavigator 的 MultiThreadedExecutor 冲突
        start = time.time()
        while not client.service_is_ready():
            if time.time() - start > 5.0:
                self._logger.error('播报服务不可用')
                return False
            time.sleep(0.1)
        
        req = SetBool.Request()
        req.data = False  # ESP32: false=舵机动作, true=重启
        # 发送即走：不等待 ESP32 响应（ESP32 会等待按钮按下，最长 60s）
        future = client.call_async(req)
        # 添加回调处理结果（不阻塞）
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
        """同步调用舵机服务，阻塞等待 ESP32 完成「播报→等按钮→返回」全流程。

        关键：使用临时独立 Node 做 service call，而不是复用 self.node（PatrolNode）。
        因为 spin_until_future_complete(PatrolNode) 会在等按钮期间密集处理
        BasicNavigator 的全部回调（17+ action client、amcl_pose、TF 等），产生
        大量 DDS 流量冲击 ESP32 的 micro-ROS UDP transport，可能导致
        /nav_node_status 消息丢失，ESP32 端 nav_node_running 一直为 false，
        触发闲时降噪关电机。

        Args:
            timeout: 等待按钮按下的超时秒数（默认 60s，与 ESP32 一致）

        Returns:
            'success' — 按钮已按下，播报完成
            'timeout' — 超时
            'servo_unavailable' — ESP32 服务不可达
            'error' — 其他异常
        """
        from rclpy.callback_groups import ReentrantCallbackGroup

        # 创建临时独立节点，避免 spin PatrolNode 期间的大量回调干扰 ESP32
        call_node = Node('servo_call_temp')
        try:
            client = call_node.create_client(
                SetBool,
                'esp32_command',
                callback_group=ReentrantCallbackGroup()
            )

            # service_is_ready() 轮询（不用 wait_for_service，避免内部 spin）
            start = time.time()
            while not client.service_is_ready():
                if time.time() - start > 5.0:
                    self._logger.error('播报服务不可用（ESP32 离线？）')
                    return 'servo_unavailable'
                time.sleep(0.1)

            req = SetBool.Request()
            req.data = False  # false=舵机动作（播报+等按钮）

            self._logger.info('📢 发送舵机播报指令，等待 ESP32 按钮确认...')
            future = client.call_async(req)

            # 只 spin 临时空节点，不干扰 PatrolNode
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
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        # 导航相关定义
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, -1.0, 4.7, 3.14,-1.0, -2.5, 0.0,-4.5, -0.55, -1.57])
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        # 实时位置获取 TF 相关定义
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        # 状态机status发布者 
        self.nav_status_pub = self.create_publisher(Bool, '/nav_node_status', 10)
        self._nav_canceled = False  # SIGINT 取消标志，供看门狗循环检查
        self.publish_nav_status(True)
        time.sleep(0.5)
    def publish_nav_status(self, status):
            """发布导航节点运行状态"""
            msg = Bool()
            msg.data = status
            self.nav_status_pub.publish(msg)
            self.get_logger().info(f'发布导航节点状态: {status}')
    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        通过 x,y,yaw 合成 PoseStamped
        """
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
        """
        通过参数值获取目标点集合        
        """
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x, y, yaw])
            self.get_logger().info(f'获取到目标点: {index}->({x},{y},{yaw})')
        return points
    def nav_to_pose(self, target_pose):
        """
        导航到指定位姿。

        BasicNavigator 没有后台 spin 线程，所有回调依赖各方法内部的
        spin_until_future_complete 驱动。isTaskComplete() 每次调用
        执行 spin_until_future_complete(timeout=0.1s) 处理回调，
        因此循环中加 time.sleep(0.1) 将轮询频率降至 ~5Hz，
        避免无 sleep 紧循环下 executor 创建/销毁过于频繁导致的竞态死锁。

        循环中每隔 ~1s 重发 nav_status=True，防止 micro-ROS/WiFi 的
        UDP 丢包导致 ESP32 端 nav_node_running 误判为 false、触发闲时降噪。
        """
        self.goToPose(target_pose)
        last_status_time = time.time()

        while not self.isTaskComplete():
            if self._nav_canceled:
                self.get_logger().info('导航已被取消')
                self.cancelTask()
                return
            # 每 ~1s 刷新 nav_status，确保 ESP32 在进入舵机等待前收到 True
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
            # 降频到 ~5Hz，避免 isTaskComplete 的 spin_until_future_complete
            # 无间隔重复调用导致 executor 竞态
            time.sleep(0.1)

        # 最终结果判断
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

    def navigate_and_complete(self, x: float, y: float, yaw: float = 0.0) -> str:
        """导航到目标点并完成舵机播报（原子操作）。

        与 nav_to_pose 相同的 isTaskComplete + sleep 模式，
        依赖 action 服务器的最终状态而非 feedback 启发式判断。

        Returns:
            'success' — 完整完成（到达 + 按钮确认）
            'canceled' — 导航被取消
            'servo_failed' — 到达了但舵机播报失败
        """
        target_pose = self.get_pose_by_xyyaw(x, y, yaw)
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
        """
        通过TF获取当前位姿。

        BasicNavigator 后台 MultiThreadedExecutor 持续 spin，
        TF 订阅回调自动填充 buffer_，因此本方法只轮询 buffer_
        不调用 spin_once（避免将 PatrolNode 临时挂到全局
        SingleThreadedExecutor 产生双重执行器冲突）。
        """
        import time
        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time) < timeout_sec:
            try:
                tf = self.buffer_.lookup_transform('map', 'base_footprint', rclpy.time.Time(seconds=0))
                transform = tf.transform

                from tf_transformations import euler_from_quaternion
                _, _, yaw = euler_from_quaternion([
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w
                ])

                self.get_logger().info(
                    f'当前位置: x={transform.translation.x:.3f}, y={transform.translation.y:.3f}, '
                    f'yaw={yaw:.3f} 弧度'
                )
                return transform

            except Exception as e:
                self.get_logger().debug(f'等待TF变换... {str(e)[:50]}')
                time.sleep(0.1)
                continue

        self.get_logger().warn(f'无法获取坐标变换（超时 {timeout_sec}s）')
        return None
def main(args=None):
    if not rclpy.ok():
            rclpy.init(args=args)    
    patrol = PatrolNode()
    
    # SIGINT 信号处理：优雅取消导航并发布状态
    import signal as sig_module
    def _sigint_handler(signum, frame):
        patrol._nav_canceled = True
        patrol.get_logger().info('收到 SIGINT，正在取消导航...')
        patrol.cancelTask()
        patrol.publish_nav_status(False)
    sig_module.signal(sig_module.SIGINT, _sigint_handler)
    
    try:
        patrol.waitUntilNav2Active()

        for point in patrol.get_target_points():
            if patrol._nav_canceled:
                break
            x, y, yaw = point[0], point[1], point[2]
            # 导航到目标点
            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            patrol.nav_to_pose(target_pose)
            # patrol.record_image()
    finally: 
        patrol.publish_nav_status(False)
        time.sleep(0.5)  # 等待消息发送完成        
        rclpy.shutdown()

if __name__ == '__main__':
    main()