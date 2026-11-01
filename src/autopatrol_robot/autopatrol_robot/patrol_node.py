import rclpy
import signal
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
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
# 导航循环的僵尸防护参数（2026-11-01）
# ------------------------------------------------------------
# nav2 掉线后 isTaskComplete() 永远不会变 True，原来的 while 会无限
# 循环、日志刷同一行 ETA，进程又不响应父进程信号（setsid），最后变成
# 刷不掉的僵尸，一直占着 /navigate_to_pose 的 action client。
# ============================================================
NAV_STALL_TIMEOUT_SEC = 20.0    # 反馈连续不变化多久判失联
NAV_MAX_WALL_SEC = 900.0        # 单次导航墙钟硬上限


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

    def call_servo_and_wait(self, timeout: float = 60.0,
                            heartbeat=None) -> str:
        """同步调用舵机服务，阻塞等待 ESP32 完成「播报→等按钮→返回」全流程。

        关键：使用临时独立 Node 做 service call，而不是复用 self.node（PatrolNode）。
        因为 spin_until_future_complete(PatrolNode) 会在等按钮期间密集处理
        BasicNavigator 的全部回调（17+ action client、amcl_pose、TF 等），产生
        大量 DDS 流量冲击 ESP32 的 micro-ROS UDP transport，可能导致
        /nav_node_status 消息丢失，ESP32 端 nav_node_running 一直为 false，
        触发闲时降噪关电机。

        Args:
            timeout: 等待按钮按下的超时秒数
            heartbeat: 可选的零参回调，在等待期间周期性调用。
                ★ 2026-11-01 新增：这 60 秒里 ESP32 处于阻塞态、停发
                  /odom，EKF 会用最后一帧速度盲推，位姿沿固定方向匀速
                  漂移（实测 55 秒漂了 20.6m）。回调用来维持
                  /nav_node_status 心跳，至少不让 ESP32 进闲时降噪。

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

            # 自己控制等待节奏，而不是一次性 spin 到底：
            # 这样每 0.5s 能调一次 heartbeat，让上层有机会维持心跳。
            deadline = time.time() + timeout
            while not future.done() and time.time() < deadline:
                rclpy.spin_until_future_complete(
                    call_node, future, timeout_sec=0.5)
                if heartbeat is not None and not future.done():
                    try:
                        heartbeat()
                    except Exception:      # noqa: BLE001
                        pass

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
        # ============================================================
        # 按钮确认的超时时间（2026-11-01 需求定型）
        # ------------------------------------------------------------
        # 语义已确定，不再是「总开关」：
        #   单点导航            → 完全不等按钮
        #   多点导航的中间点    → 等按钮（本超时），超时则取消整个任务
        #   多点导航的最后一个点 → 不等
        #
        # 等待期间 ESP32 会阻塞 micro-ROS 任务、停发 /odom 和 /imu_raw，
        # 所以必须先 /odom_freeze 冻住位姿（见 _wait_button_confirm）。
        #
        # 超时时间保持 60 秒（与 ESP32 固件内部 timeout 一致）。
        # 比它小会提前解冻，但 ESP32 那边还在等 —— 状态不一致。
        # ============================================================
        self.declare_parameter('servo_timeout_sec', 60.0)
        self.servo_timeout_sec_ = float(
            self.get_parameter('servo_timeout_sec').value)
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        # 实时位置获取 TF 相关定义
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        # ★ 直接订阅 /amcl_pose 并缓存，供 get_current_position() 使用。
        #
        # 原实现每次查位置都 spawn 一个 `ros2 topic echo --once` 子进程：
        #   - 启动子进程本身就要 0.5~1s，最坏要等 5s 超时
        #   - 依赖子进程能拿到正确的 ROS 环境（该子进程继承 self 的环境，
        #     实际能用，但一旦 web 服务从异常环境启动就敲不进）
        #   - AI 连续问几次位置就卡几秒，体验极差
        # 改成订阅缓存后，查询是纯内存读取，微秒级。
        #
        # 用 RELIABLE 订阅：AMCL 以 RELIABLE 发 /amcl_pose。
        self._amcl_pose = None          # 最新位姿 {'x','y','yaw'}
        self._amcl_pose_stamp = None    # 收到该位姿的时间（time.time()）
        try:
            self.amcl_pose_sub = self.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose',
                self._on_amcl_pose, 10)
        except Exception as exc:        # noqa: BLE001 - 缺这层不该阻断节点
            self.amcl_pose_sub = None
            self.get_logger().warn(
                f'订阅 /amcl_pose 失败，位置查询将回退到 TF: {exc}')
        # 状态机status发布者 
        self.nav_status_pub = self.create_publisher(Bool, '/nav_node_status', 10)
        # 位姿冻结开关（见 set_odom_freeze 的注释）
        self._freeze_pub = self.create_publisher(Bool, '/odom_freeze', 10)
        self._nav_canceled = False  # SIGINT 取消标志，供看门狗循环检查
        # 当前状态缓存（心跳用）。None 表示尚未初始化，心跳不发送。
        self._nav_status_current = None
        self.publish_nav_status(True)
        # ★ 心跳定时器：每 1 秒重发一次当前状态。
        # 为什么必须加：/nav_node_status 被下游当作「导航是否在进行」的门控
        # 开关使用（ESP32 的闲时降噪、fishbot_voice 的语音门控）。
        # 如果只在状态变化时发一次，那么一旦本节点被强杀 / 异常退出 / 被
        # web_controller 的 force_cleanup_all() pkill 掉，那条 false 永远
        # 发不出去，下游就会永久卡在「导航中」：
        #   - 语音控制静默失效（运动指令被门控丢弃），只有重启语音节点才恢复
        #   - ESP32 不再自动停车（安全隐患）
        # 有了心跳，下游就能用「超时未更新」安全判定导航已停止。
        self._nav_status_timer = self.create_timer(
            1.0, self._nav_status_heartbeat)
        time.sleep(0.5)

    def publish_nav_status(self, status):
            """发布导航节点运行状态（同时更新心跳缓存）"""
            self._nav_status_current = bool(status)
            msg = Bool()
            msg.data = self._nav_status_current
            self.nav_status_pub.publish(msg)
            self.get_logger().info(f'发布导航节点状态: {self._nav_status_current}')

    def _nav_status_heartbeat(self):
            """周期重发当前状态，充当心跳。未初始化时不发。"""
            if self._nav_status_current is None:
                return
            msg = Bool()
            msg.data = self._nav_status_current
            self.nav_status_pub.publish(msg)

    # ------------------------------------------------------------------
    # AMCL 位姿缓存
    # ------------------------------------------------------------------
    def _on_amcl_pose(self, msg):
        """缓存最新 AMCL 位姿（回调里只做最轻量的转换）。"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # yaw = atan2(2(wz+xy), 1-2(y²+z²))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        import math
        self._amcl_pose = {
            'x': float(p.x),
            'y': float(p.y),
            'yaw': math.atan2(siny_cosp, cosy_cosp),
        }
        self._amcl_pose_stamp = time.time()

    def get_cached_amcl_pose(self, max_age: float = 3.0):
        """返回缓存的 AMCL 位姿；无数据或太旧则返回 None。"""
        if self._amcl_pose is None or self._amcl_pose_stamp is None:
            return None
        if time.time() - self._amcl_pose_stamp > max_age:
            return None
        return dict(self._amcl_pose)

    def get_cached_amcl_pose_raw(self):
        """返回 (位姿字典或None, 距上一帧的秒数或None)。"""
        if self._amcl_pose is None or self._amcl_pose_stamp is None:
            return None, None
        return dict(self._amcl_pose), time.time() - self._amcl_pose_stamp

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
    # ============================================================
    # 位姿冻结（2026-11-01）
    # ------------------------------------------------------------
    # ESP32 的 esp32_command 在「等按钮」模式下会**阻塞 micro-ROS 任务
    # 最多 60 秒**，期间 /odom 和 /imu_raw 全部停发。EKF 收不到观测
    # 就按上一帧速度继续积分 —— 实测 55 秒把 odom→base_footprint 从
    # 0.2m 推到 20.6m（方向固定 53°，速度恒定 0.37m/s），AMCL 匹配率
    # 从 92% 掉到 16~37%。用户看到的就是「位姿缓慢漂移」。
    #
    # 现在：进按钮等待前先发 /odom_freeze=true，让 odom2tf 锁住位姿；
    # 等完再发 false 解冻。odom/imu 可以照常停发，但 TF 不再动。
    # ============================================================
    def set_odom_freeze(self, freeze: bool) -> None:
        """请求 odom2tf 冻结/解冻 odom→base_footprint。"""
        try:
            if self._freeze_pub is None:
                return
            msg = Bool()
            msg.data = bool(freeze)
            # 连发 3 次：这个切换很关键，丢一帧就会白等 60 秒。
            # ROS 的 QoS 只在队列满时丢，连发成本极低。
            for _ in range(3):
                self._freeze_pub.publish(msg)
            self.get_logger().info(
                '🧊 %s位姿冻结' % ('开始' if freeze else '结束'))
        except Exception as exc:            # noqa: BLE001
            self.get_logger().warn('位姿冻结请求失败（不影响流程）: %s' % exc)

    def nav_to_pose(self, target_pose, confirm_button: bool = False) -> bool:
        """导航到指定位姿。

        BasicNavigator 没有后台 spin 线程，所有回调依赖各方法内部的
        spin_until_future_complete 驱动。isTaskComplete() 每次调用
        执行 spin_until_future_complete(timeout=0.1s) 处理回调，
        因此循环中加 time.sleep(0.1) 将轮询频率降至 ~5Hz，
        避免无 sleep 紧循环下 executor 创建/销毁过于频繁导致的竞态死锁。

        循环中每隔 ~1s 重发 nav_status=True，防止 micro-ROS/WiFi 的
        UDP 丢包导致 ESP32 端 nav_node_running 误判为 false、触发闲时降噪。

        Args:
            target_pose: 目标位姿
            confirm_button: 到达后是否**阻塞等待按钮确认**。
                ★ 2026-11-01 需求：只有「多点导航的中间点」才需要确认；
                  单点导航和最后一个点都不等（用户明确要求）。

        Returns:
            True  — 到达（需要确认时，按钮已按下）
            False — 导航失败 / 被取消 / 按钮等待超时
        """
        self.goToPose(target_pose)
        last_status_time = time.time()

        # ============================================================
        # 僵尸防护（2026-11-01 实机事故）
        # ------------------------------------------------------------
        # nav2 被关掉后，action goal 永远没有 result，isTaskComplete()
        # 永远返回 False → 这里会无限循环。实测 nav_to_pose 就这样卡了
        # 二十多分钟，日志里每秒刷同一行 ETA，父进程重启也管不到它
        # （preexec_fn=os.setsid 让它独立成会话组）。
        #
        # 两道保险：反馈长时间不变化 → 判失联；墙钟硬上限。
        # ============================================================
        wall_start = time.time()
        last_fb_sig = None
        last_fb_change = time.time()

        while not self.isTaskComplete():
            if self._nav_canceled:
                self.get_logger().info('导航已被取消')
                self.cancelTask()
                return False
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
                sig = (round(remaining, 2),
                       round(float(feedback.distance_remaining), 2))
                if last_fb_sig is None or sig != last_fb_sig:
                    last_fb_sig = sig
                    last_fb_change = now
                self.get_logger().info(
                    f'预计剩余: {remaining:.1f}s, '
                    f'距离: {feedback.distance_remaining:.2f}m'
                )
            # 保险 1：反馈长时间不变化 → Nav2 大概率掉线
            if now - last_fb_change > NAV_STALL_TIMEOUT_SEC:
                self.get_logger().error(
                    f'⚠️ 反馈已 {NAV_STALL_TIMEOUT_SEC:.0f}s 无变化，'
                    f'判定与 Nav2 失联，主动退出（避免僵尸进程）')
                self.cancelTask()
                return False
            # 保险 2：墙钟总超时
            if now - wall_start > NAV_MAX_WALL_SEC:
                self.get_logger().error(
                    f'⚠️ 导航已运行 {NAV_MAX_WALL_SEC:.0f}s 仍无结果，主动退出')
                self.cancelTask()
                return False
            # 降频到 ~5Hz，避免 isTaskComplete 的 spin_until_future_complete
            # 无间隔重复调用导致 executor 竞态
            time.sleep(0.1)

        # 最终结果判断
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航结果：成功')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('导航结果：已取消')
            return False
        else:
            self.get_logger().warn(f'导航结果异常: {result}')
            return False

        if not confirm_button:
            # 单点 / 最后一个点：直接结束，不播报、不等按钮
            return True

        return self._wait_button_confirm('目标点')

    def _wait_button_confirm(self, label: str = '') -> bool:
        """到达中间航点后，阻塞等待按钮确认（**期间冻结位姿**）。

        ============================================================
        适用场景（用户 2026-11-01 明确的需求）
        ============================================================
            单点导航          → 不等按钮
            「先去A再去B」    → 到 A 等按钮；到 B（最后一个）不等
            等按钮超时        → **取消整个任务**

        ============================================================
        为什么必须先冻结
        ============================================================
        ESP32 在这个服务里会阻塞 micro-ROS 任务最多 60 秒，期间
        /odom 和 /imu_raw 全部停发。EKF 收不到观测就按上一帧速度
        继续积分 —— 实测 55 秒漂了 20.6 米。

        所以顺序必须是：
            1. 先发 /odom_freeze=true（锁住 TF）
            2. 再调 esp32_command
            3. 无论成功/超时/异常，finally 里一定解冻
        顺序反了或者漏了解冻，都会留下一个冻住的 TF，整台车“失联”。

        Returns:
            True  — 按钮已按下
            False — 超时/异常（调用方应当取消整个任务）
        """
        self.get_logger().info('⏸️ 已到达%s，等待按钮确认后才能继续' % (label or ''))
        self.set_odom_freeze(True)
        try:
            servo = ServoClient(node=self)
            servo.stop_robot()
            time.sleep(0.3)
            result = servo.call_servo_and_wait(
                timeout=self.servo_timeout_sec_,
                heartbeat=self._heartbeat_during_servo)
        finally:
            # ★ finally：哪怕中间抛异常也必须解冻，否则 TF 永远卡住
            self.set_odom_freeze(False)

        if result == 'success':
            self.get_logger().info('✅ 按钮已按下，继续前往下一航点')
            return True
        if result == 'timeout':
            self.get_logger().error(
                '❌ 等待按钮超时（%.0fs），按需求取消整个导航任务'
                % self.servo_timeout_sec_)
        else:
            self.get_logger().error(
                '❌ 按钮确认失败（%s），取消整个导航任务' % result)
        return False

    def _heartbeat_during_servo(self) -> None:
        """按钮等待期间的心跳。

        位姿已经靠 /odom_freeze 冻住了，这里只维持
        /nav_node_status = True，告诉 ESP32“导航节点还在”，
        避免它 3 秒后进闲时降噪。
        """
        try:
            self.publish_nav_status(True)
        except Exception:          # noqa: BLE001 - 心跳失败不影响主流程
            pass

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

        # 僵尸防护，同 nav_to_pose（详见顶部常量注释）
        wall_start = time.time()
        last_fb_sig = None
        last_fb_change = time.time()

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
                sig = (round(remaining, 2),
                       round(float(feedback.distance_remaining), 2))
                if last_fb_sig is None or sig != last_fb_sig:
                    last_fb_sig = sig
                    last_fb_change = now
                self.get_logger().info(
                    f'预计剩余: {remaining:.1f}s, '
                    f'距离: {feedback.distance_remaining:.2f}m'
                )
            if now - last_fb_change > NAV_STALL_TIMEOUT_SEC:
                self.get_logger().error(
                    f'⚠️ 反馈已 {NAV_STALL_TIMEOUT_SEC:.0f}s 无变化，'
                    f'判定与 Nav2 失联，主动退出（避免僵尸进程）')
                self.cancelTask()
                return 'servo_failed'
            if now - wall_start > NAV_MAX_WALL_SEC:
                self.get_logger().error(
                    f'⚠️ 导航已运行 {NAV_MAX_WALL_SEC:.0f}s 仍无结果，主动退出')
                self.cancelTask()
                return 'servo_failed'
            time.sleep(0.1)

        result = self.getResult()
        if result == TaskResult.CANCELED:
            self.get_logger().info('导航已被取消')
            return 'canceled'
        if result != TaskResult.SUCCEEDED:
            self.get_logger().warn(f'导航异常: {result}')
            return 'servo_failed'

        self.get_logger().info('导航到达目标点')
        # 这里是「单点导航」的原子接口，按需求**不等按钮**（用户 2026-11-01）。
        # 多点序列的中间点确认由 main() → nav_to_pose(confirm_button=True) 负责。
        self.get_logger().info('✅ 导航完成')
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

        # ============================================================
        # 多点导航：中间点等按钮确认，最后一个点不等（2026-11-01 需求）
        # ------------------------------------------------------------
        # 用户原话：
        #   「单点导航完成后无需强制阻塞等待按按钮确认，只有两个点或
        #     以上的需要按按钮确认。比如我向 ai 助手下达：先去书房、
        #     再去客厅。此时小车到达书房后需阻塞等待按钮被按下而后
        #     前往下一航点，若等待超时则直接取消整个任务，但全程小车
        #     位姿要保持稳定。」
        #
        # 所以：points[0..n-2] → confirm_button=True
        #       points[n-1]     → confirm_button=False（任务结束）
        #   单点（n==1）自然全部不等。
        #
        # 按钮等待期间位姿由 /odom_freeze 冻住，odom/imu 可以停发，
        # TF 停在到达时的快照上，不会漂。
        # ============================================================
        points = patrol.get_target_points()
        n = len(points)
        if n <= 1:
            patrol.get_logger().info(
                '单点导航：到达即结束，不等待按钮确认')
        else:
            patrol.get_logger().info(
                '多点导航：共 %d 个航点，前 %d 个到达后需按钮确认，'
                '最后一个直接结束' % (n, n - 1))

        for idx, point in enumerate(points):
            if patrol._nav_canceled:
                break
            x, y, yaw = point[0], point[1], point[2]
            is_last = (idx == n - 1)
            needs_confirm = (not is_last)          # 单点时 n-1==0，永远 False
            patrol.get_logger().info(
                '▶️ 第 %d/%d 个航点 (%.2f, %.2f, %.2f)%s'
                % (idx + 1, n, x, y, yaw,
                   '' if is_last else '，到达后需按钮确认'))

            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            ok = patrol.nav_to_pose(target_pose,
                                    confirm_button=needs_confirm)
            if not ok:
                # 导航失败 / 被取消 / 按钮超时 → 取消整个任务
                patrol.get_logger().error(
                    '❌ 第 %d/%d 个航点未完成，取消整个导航任务'
                    % (idx + 1, n))
                break
            # patrol.record_image()
    finally: 
        patrol.publish_nav_status(False)
        time.sleep(0.5)  # 等待消息发送完成        
        rclpy.shutdown()

if __name__ == '__main__':
    main()