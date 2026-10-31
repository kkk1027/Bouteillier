#!/usr/bin/env python3
"""voice_control_node.py
====================================================
语音控制节点 —— 本包唯一的可执行节点。

【职责边界】
  输入：/voice_event        (std_msgs/String)  ASR-01 上报的原始文本，如 "MOVE_FWD"
        /voice_module_status (std_msgs/Bool)   语音模块在线状态
        /nav_node_status     (std_msgs/Bool)   导航节点是否运行（门控用）
  输出：/cmd_vel            (geometry_msgs/Twist)

  本节点只做「语音文本 -> 底盘速度」的翻译与执行。
  不负责：串口读取（在 ESP32 固件里）、导航、建图、Web 交互、语音播报。

【为什么固件不解析命令】
  ESP32 只上报原始文本，命令词表与动作策略全部在本节点。
  改一条语音逻辑只需重启本节点（约 1 秒），不用重新编译烧录固件（约 30 秒）。

【控制模型：定时脉冲，不是单次下发】
  ESP32 固件有「3 秒无 /cmd_vel 即自动停车」的保护（防上位机崩溃后小车
  冲出去）。因此一次动作必须持续以 publish_rate 连发 Twist，只发一次车会
  提前停住。

【安全优先级】
  STOP（FOLLOW_OFF）最高优先级，任何时候都立即生效，不受去抖和门控限制。
  其余动作可互相打断（后到的覆盖先到的）。
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

# ---------------------------------------------------------------------------
# 动作枚举
# ---------------------------------------------------------------------------
A_NONE = 'NONE'              # 仅记录日志，不动作（如 WAKE 唤醒词）
A_FORWARD = 'FORWARD'
A_BACKWARD = 'BACKWARD'
A_TURN_LEFT = 'TURN_LEFT'
A_TURN_RIGHT = 'TURN_RIGHT'
A_STOP = 'STOP'              # 立即停车，并解除当前动作

LINEAR_ACTIONS = {A_FORWARD, A_BACKWARD}
TURN_ACTIONS = {A_TURN_LEFT, A_TURN_RIGHT}
MOTION_ACTIONS = LINEAR_ACTIONS | TURN_ACTIONS

# 各动作的运动签名 (linear_sign, angular_sign)。
# 用于判断「新动作与当前动作方向是否冲突」——冲突时先停车再切换，
# 避免前进中直接反转电机（损伤减速箱）或直线中突然原地转（打滑）。
_ACTION_SIGNATURE = {
    A_FORWARD: (1, 0),
    A_BACKWARD: (-1, 0),
    A_TURN_LEFT: (0, 1),
    A_TURN_RIGHT: (0, -1),
}

# 默认命令词表：ASR-01 模块端输出的文本 -> 动作
DEFAULT_COMMAND_MAP = [
    'WAKE:NONE',
    'MOVE_FWD:FORWARD',
    'MOVE_BACK:BACKWARD',
    'MOVE_LEFT:TURN_LEFT',
    'MOVE_RIGHT:TURN_RIGHT',
    'FOLLOW_OFF:STOP',
]


class VoiceControlNode(Node):
    """语音事件 -> /cmd_vel 脉冲控制的执行器。"""

    def __init__(self):
        super().__init__('voice_control_node')

        # ---------------- 参数声明 ----------------
        # 速度与时长：动作 = 速度 x 时长。0.2 m/s * 5 s = 1 米，
        # 室内请按实际场地调整，改 yaml 即可，不用改代码。
        self.declare_parameter('linear_speed', 0.2)      # m/s
        # 原地转向角速度。角度 = angular_speed x action_duration
        #   1.0 rad/s x 5s = 5.0 rad = 286°
        self.declare_parameter('angular_speed', 1.0)     # rad/s
        self.declare_parameter('action_duration', 5.0)   # s
        # 发布频率：必须足够高。ESP32 侧 3 秒无 Twist 会自动停车，
        # 20Hz 兼顾实时性与 WiFi 带宽。
        self.declare_parameter('publish_rate', 20.0)     # Hz
        # 方向切换前的停车过渡时长（秒）。0 表示不做过渡。
        self.declare_parameter('pre_stop_duration', 0.2)  # s
        # 动作结束后连发几次零速度，确保 ESP32 一定收到停止指令
        self.declare_parameter('stop_burst', 3)          # 次数
        # 同命令最小重复间隔（秒），滤掉模块重复上报或回声误触发。
        # STOP 不受此限制。
        self.declare_parameter('min_repeat_interval', 0.4)  # s
        # 导航运行中是否允许语音运动指令覆盖。
        # False = 忽略 MOVE_*（推荐，避免和 Nav2 抢 /cmd_vel），STOP 仍生效。
        self.declare_parameter('allow_motion_during_nav', False)
        # ★ 导航状态看门狗（关键！）
        # /nav_node_status 是无锁存的瞬时话题，nav 节点被 Ctrl+C 杀掉时
        # 不会再补发一个 false。若只看最后一条消息，_nav_running 会永久
        # 停在 True，导致此后所有语音运动指令被静默忽略、
        # 只有重启节点才恢复——这就是「语音偶发失效」的真正原因。
        # 超过这个秒数没收到新消息，就认为导航已经停止。
        # patrol_node / nav_to_pose 均以 1Hz 发心跳，3s = 容许丢 2 帧。
        self.declare_parameter('nav_status_timeout', 3.0)
        # 命令词表，格式 "文本:动作"，动作取值见文件顶部常量
        self.declare_parameter('command_map', DEFAULT_COMMAND_MAP)
        # ★★★ 安全开关（总闸）★★★
        # False（默认）= 干跑模式：所有语音指令正常解析、正常记日志、正常走状态机，
        #   但【绝不向 /cmd_vel 发布任何数据包】，车一定不会动。
        # True = 实车模式：真正下发速度。
        # 为什么默认 False：只要本节点在跑，喊一句话车就会动。开发调试、改参数、
        # 联调话题时绝不应该有这种风险。确认一切无误后再显式打开。
        self.declare_parameter('enable_motion', False)

        # ---------------- 读取参数 ----------------
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.action_duration = float(self.get_parameter('action_duration').value)
        rate = float(self.get_parameter('publish_rate').value)
        self.pre_stop_duration = float(self.get_parameter('pre_stop_duration').value)
        self.stop_burst = int(self.get_parameter('stop_burst').value)
        self.min_repeat_interval = float(self.get_parameter('min_repeat_interval').value)
        self.allow_motion_during_nav = bool(
            self.get_parameter('allow_motion_during_nav').value)
        self.nav_status_timeout = float(
            self.get_parameter('nav_status_timeout').value)
        if self.nav_status_timeout <= 0.0:
            self.nav_status_timeout = 3.0

        self.command_map = self._parse_command_map(
            self.get_parameter('command_map').value)
        self.enable_motion = bool(
            self.get_parameter('enable_motion').value)
        self._dry_last_emit = None   # 干跑模式下用于日志去重，避免 20Hz 刷屏

        if rate <= 0.0:
            self.get_logger().warn('publish_rate <= 0，回退到 20Hz')
            rate = 20.0
        self.timer_period = 1.0 / rate

        # ---------------- 参数合法性检查 ----------------
        # 速度/时长直接决定车跑多远，配错会撞墙，必须拦。
        if self.linear_speed < 0.0:
            self.get_logger().warn(
                f'linear_speed 为负 ({self.linear_speed})，按绝对值处理')
            self.linear_speed = abs(self.linear_speed)
        if self.angular_speed < 0.0:
            self.get_logger().warn(
                f'angular_speed 为负 ({self.angular_speed})，按绝对值处理')
            self.angular_speed = abs(self.angular_speed)
        if self.action_duration <= 0.0:
            self.get_logger().warn(
                f'action_duration <= 0 ({self.action_duration})，回退到 5.0s')
            self.action_duration = 5.0
        if self.pre_stop_duration < 0.0:
            self.pre_stop_duration = 0.0
        if self.stop_burst < 0:
            self.stop_burst = 0
        # 动作距离告警：距离 = 速度 x 时长
        travel = self.linear_speed * self.action_duration
        if travel > 1.5:
            self.get_logger().warn(
                f'单次直行距离 {travel:.2f} m 偏大，请确认前方空间足够')
        if self.linear_speed > 0.5:
            self.get_logger().warn(
                f'linear_speed {self.linear_speed} m/s 偏高，建议 0.2 以下')

        # ---------------- 运行时状态 ----------------
        self._action = A_NONE          # 当前正在执行的动作
        self._action_end = 0.0         # 当前动作的结束时刻（time.monotonic）
        self._pre_stop_until = 0.0     # 方向切换过渡的结束时刻
        self._pending_action = A_NONE  # 过渡结束后要执行的动作
        self._stop_burst_left = 0      # 还需连发几次零速度
        self._last_cmd_text = ''       # 上一条命令文本（去重用）
        self._last_cmd_time = 0.0
        self._nav_running = False
        self._nav_last_msg_time = 0.0   # 最后一条 /nav_node_status 的到达时刻
        self._nav_timeout_warned = False
        self._module_online = False

        # ---------------- QoS ----------------
        # 关键：ESP32 的 micro-ROS 发布器用的是 best_effort，
        # rclpy 侧如果用默认的 RELIABLE 会因 QoS 不兼容而收不到任何数据。
        # 这类问题不会报错，只表现为「话题存在但收不到」，最难排查。
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # /cmd_vel 也必须兼容 ESP32 侧的 best_effort 订阅
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------------- 接口 ----------------
        self.create_subscription(
            String, '/voice_event', self._on_voice_event, qos_sensor)
        self.create_subscription(
            Bool, '/voice_module_status', self._on_module_status, qos_sensor)
        # /nav_node_status 由 patrol_node 用默认 QoS(RELIABLE) 发布，
        # 这里用默认 QoS 匹配
        self.create_subscription(
            Bool, '/nav_node_status', self._on_nav_status, 10)

        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', qos_cmd)
        self._timer = self.create_timer(self.timer_period, self._on_timer)

        self.get_logger().info('=' * 60)
        self.get_logger().info('voice_control_node 已启动')
        if self.enable_motion:
            self.get_logger().warn(
                '★ 安全开关 enable_motion = True —— 实车模式，语音指令会真的驱动底盘')
        else:
            self.get_logger().info(
                '★ 安全开关 enable_motion = False —— 干跑模式，'
                '只解析和打日志，不会向 /cmd_vel 发布任何数据')
            self.get_logger().info(
                '  确认无误后加参数启动实车：'
                'ros2 launch fishbot_voice voice_control.launch.py enable_motion:=true')
        self.get_logger().info(
            f'  线速度 {self.linear_speed} m/s  角速度 {self.angular_speed} rad/s')
        self.get_logger().info(
            f'  动作时长 {self.action_duration} s  发布频率 {rate:.0f} Hz')
        self.get_logger().info(f'  命令词表（共 {len(self.command_map)} 条）:')
        for text, act in self.command_map.items():
            self.get_logger().info(f'    {text:14s} -> {act}')
        self.get_logger().info(
            f'  导航中允许语音运动: {self.allow_motion_during_nav}')
        self.get_logger().info(
            '  提示：可用 ros2 topic pub /voice_event std_msgs/String '
            '"{data: MOVE_FWD}" 空跑测试，不用喊话')
        self.get_logger().info('=' * 60)

    # ------------------------------------------------------------------
    # 参数解析
    # ------------------------------------------------------------------
    @staticmethod
    def _parse_command_map(entries):
        """把 ["文本:动作", ...] 解析成 dict。非法项跳过并告警。"""
        mapping = {}
        valid = {A_NONE, A_FORWARD, A_BACKWARD, A_TURN_LEFT, A_TURN_RIGHT, A_STOP}
        for raw in entries:
            if not isinstance(raw, str) or ':' not in raw:
                continue
            text, _, act = raw.partition(':')
            text = text.strip()
            act = act.strip().upper()
            if not text:
                continue
            if act not in valid:
                continue
            mapping[text.upper()] = act
        return mapping

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------
    def _on_voice_event(self, msg: String):
        text = (msg.data or '').strip()
        if not text:
            return

        action = self.command_map.get(text.upper())
        if action is None:
            self.get_logger().warn(f'未识别的语音文本: "{text}"（请检查 command_map 参数）')
            return

        now = time.monotonic()

        # ---- STOP 最高优先级：不受去抖、不受导航门控 ----
        if action == A_STOP:
            self._do_stop(now, reason=f'语音 "{text}"')

        # ---- 去抖：同一条文本短时间内重复到达直接丢弃 ----
        elif (text.upper() == self._last_cmd_text
              and (now - self._last_cmd_time) < self.min_repeat_interval):
            self.get_logger().debug(f'去抖丢弃: "{text}"')

        # ---- 导航门控：导航运行时忽略运动指令，避免抢 /cmd_vel ----
        # 带看门狗：nav 节点被强杀时不会再发 false，超时后自动解锁
        elif self._nav_is_blocking(now):
            self.get_logger().warn(
                f'忽略 "{text}" -> {action}：导航节点运行中（allow_motion_during_nav=false）')

        # ---- 纯唤醒词等无动作命令 ----
        elif action == A_NONE:
            self.get_logger().info(f'语音 "{text}" -> 仅唤醒，不动作')

        # ---- 正常运动指令 ----
        else:
            self._last_cmd_text = text.upper()
            self._last_cmd_time = now
            self._start_action(action, now, source=f'语音 "{text}"')

    def _on_module_status(self, msg: Bool):
        online = bool(msg.data)
        if online == self._module_online:
            return
        self._module_online = online
        if online:
            self.get_logger().info('语音模块上线')
        else:
            # 模块离线不打断正在执行的动作——动作有固定时长，
            # 中途停反而可能停在危险位置。只记录日志。
            self.get_logger().warn('语音模块离线（3 秒无数据），请检查供电与串口')

    def _on_nav_status(self, msg: Bool):
        running = bool(msg.data)
        self._nav_last_msg_time = time.monotonic()
        if running == self._nav_running:
            return
        self._nav_running = running
        self.get_logger().info(f'导航节点{"启动" if running else "停止"}')

    def _nav_is_blocking(self, now):
        """导航是否正在阻挡语音运动指令（带看门狗）。

        必须检查消息新鲜度：nav 节点被强杀时不会再发 false，
        只看 _nav_running 会永久卡在 True，语音就“忽然不灵了”。
        """
        if not self._nav_running or self.allow_motion_during_nav:
            return False
        if now - self._nav_last_msg_time > self.nav_status_timeout:
            # 超时未更新 -> 认为 nav 已不在，自动解锁（只告警一次）
            if not self._nav_timeout_warned:
                self._nav_timeout_warned = True
                self.get_logger().warn(
                    f'/nav_node_status 已 {self.nav_status_timeout:.1f}s '
                    f'无更新，判定导航已停止，解除语音门控')
            return False
        self._nav_timeout_warned = False
        return True

    # ------------------------------------------------------------------
    # 动作执行
    # ------------------------------------------------------------------
    def _start_action(self, action, now, source=''):
        """启动一个定时动作，必要时先插入停车过渡。"""
        # 方向冲突判定：与当前动作的运动签名不同 -> 先停车再切换
        need_pre_stop = (
            self._action in MOTION_ACTIONS
            and _ACTION_SIGNATURE.get(action) != _ACTION_SIGNATURE.get(self._action)
            and self.pre_stop_duration > 0.0
        )

        if need_pre_stop:
            self._pre_stop_until = now + self.pre_stop_duration
            self._pending_action = action
            self._action = A_NONE
            self._action_end = 0.0
            # 清掉上一个动作残留的停车补发计数，
            # 否则它会在本次动作结束后额外多发几帧零速度
            self._stop_burst_left = 0
            self.get_logger().info(
                f'{source} -> {action}：先停车 {self.pre_stop_duration:.2f}s 再切换方向')
            return

        self._pre_stop_until = 0.0
        self._pending_action = A_NONE
        self._action = action
        self._action_end = now + self.action_duration
        self._stop_burst_left = 0
        self.get_logger().info(
            f'{source} -> {action} 执行 {self.action_duration:.1f}s')

    def _do_stop(self, now, reason=''):
        """立即停车并清空所有待执行动作。"""
        was_active = self._action in MOTION_ACTIONS or self._pending_action in MOTION_ACTIONS
        self._action = A_NONE
        self._action_end = 0.0
        self._pre_stop_until = 0.0
        self._pending_action = A_NONE
        # 连发几次零速度，确保 ESP32 一定收到（UDP best_effort 可能丢包）
        self._stop_burst_left = self.stop_burst
        self._emit(0.0, 0.0)
        if self.stop_burst > 0:
            self._stop_burst_left -= 1
        if was_active:
            self.get_logger().info(f'{reason} -> 停车（已中断动作）')
        else:
            self.get_logger().info(f'{reason} -> 停车')

    # ------------------------------------------------------------------
    # 速度下发（唯一出口，安全开关在这里生效）
    # ------------------------------------------------------------------
    def _publish_twist(self, linear, angular):
        """底层发布。调用方应优先用 _emit()，它会处理安全开关。"""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._pub_cmd.publish(msg)

    def _emit(self, linear, angular):
        """所有速度下发的统一出口。

        【安全开关】enable_motion=False 时一个数据包都不发，绝不碰 /cmd_vel：
          不只是拦截非零速度，零速度也不发。原因：
            - 零速度虽无害，但会让 ESP32 刷新 last_cmd_vel_time，
              干扰其他正在控制底盘的节点（如 Nav2）。
            - 「完全没有发布者」比「发布零速」状态更干净、更好排查。
          干跑时只在速度值变化的那一刻打一行日志，不会 20Hz 刷屏。
        """
        if self.enable_motion:
            self._publish_twist(linear, angular)
            return

        key = (round(linear, 3), round(angular, 3))
        if key != self._dry_last_emit:
            self._dry_last_emit = key
            self.get_logger().info(
                f'[干跑] 本应下发 linear={linear:+.2f} angular={angular:+.2f}'
                f' —— 已拦截，未发布任何数据')

    def has_active_motion(self) -> bool:
        """当前是否有本节点发起的、尚未结束的动作。"""
        return (self._action in MOTION_ACTIONS
                or self._pending_action in MOTION_ACTIONS)

    def publish_stop(self):
        """立即下发零速度（公开方法，供退出流程调用）。"""
        self._emit(0.0, 0.0)

    def _action_to_twist(self, action):
        """动作 -> 速度指令。"""
        if action == A_FORWARD:
            return self.linear_speed, 0.0
        if action == A_BACKWARD:
            return -self.linear_speed, 0.0
        if action == A_TURN_LEFT:
            # 原地转：只给角速度，线速度必须为 0
            return 0.0, self.angular_speed
        if action == A_TURN_RIGHT:
            return 0.0, -self.angular_speed
        return 0.0, 0.0

    # ------------------------------------------------------------------
    # 主循环（定时器）
    # ------------------------------------------------------------------
    def _on_timer(self):
        now = time.monotonic()

        # 1) 停车过渡中：发零速度，到期后启动待执行动作
        if self._pre_stop_until > 0.0:
            if now < self._pre_stop_until:
                self._emit(0.0, 0.0)
                return
            pending = self._pending_action
            self._pre_stop_until = 0.0
            self._pending_action = A_NONE
            if pending in MOTION_ACTIONS:
                self._action = pending
                self._action_end = now + self.action_duration
                self.get_logger().info(
                    f'{pending} 开始执行 {self.action_duration:.1f}s')
                self._emit(*self._action_to_twist(pending))
                return

        # 2) 动作执行中：到期则收尾
        if self._action in MOTION_ACTIONS:
            if now < self._action_end:
                self._emit(*self._action_to_twist(self._action))
                return
            done = self._action
            self._action = A_NONE
            self._action_end = 0.0
            self._stop_burst_left = self.stop_burst
            self.get_logger().info(f'{done} 结束 -> 停车')

        # 3) 停车补发：动作刚结束，多补几帧零速度防止丢包
        if self._stop_burst_left > 0:
            self._stop_burst_left -= 1
            self._emit(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前必须保证底盘停下 —— 但只在「本节点确实有动作在跑」时才发零速。
        # 无条件发零速会有副作用：如果此时 Nav2 正在驱动底盘，本节点退出会
        # 把导航中的车顺手刹停。
        try:
            if node.has_active_motion():
                node.publish_stop()
                node.get_logger().info('退出前下发零速度')
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
