#!/usr/bin/env python3
"""
amcl_tracer.py
AMCL 重定位过程记录器 —— 随导航栈自动拉起，**只读，不发任何指令**。

================================================================
解决什么问题
================================================================
实车出现「位姿突然严重抖动、大幅跳变」，但现象偶发、事后无从复现。
之前排查只能靠临时采样几分钟，抓不到就白等；而且 nav 栈一重启，
之前那次的证据就全丢了。

本节点随导航栈自动启动，全程把 AMCL 的状态流写到磁盘，
出问题时直接看文件即可回溯。

================================================================
记录什么（CSV，每行一个采样点）
================================================================
    t            启动后秒数
    wall         墙上时间
    x y yaw      AMCL 位姿（map 系）
    std_xy       位置标准差（由协方差折算）
    std_yaw      朝向标准差（rad）
    mo_x mo_y     map→odom（纯 AMCL 修正量）
    mo_yaw
    ob_x ob_y    odom→base_footprint（纯 EKF 里程计）
    ob_yaw
    d_pose       相对上一采样点的位置变化（m）
    d_yaw        朝向变化（度）
    d_mo         map→odom 变化（m）
    d_ob         odom→base 变化（m）
    state        判定状态：TRACK / CONVERGE / COLLAPSE / DIVERGE / NO_INPUT
    evt          本行附带事件（JUMP / RESEED / …）

================================================================
为什么能区分跳变来源
================================================================
    AMCL 最终位姿 = map→odom ∘ odom→base_footprint
    所以：
      * map→odom 跳、odom→base 稳  → AMCL 在跳（重定位/粒子问题）
      * odom→base 跳、map→odom 稳  → EKF / 编码器问题
      * 两者都跳                    → 上游数据问题

================================================================
重定位事件怎么捕获
================================================================
auto_localizer 每次「重新撒粒子」都会往 /initialpose 发一条消息
（auto_localizer.py:516）。所以订阅 /initialpose 就能精确计数
重定位次数和时刻，不需要读它的内部状态。

同时本节点会镜像 auto_localizer 的塌缩判据
（std_xy < 0.005 且 std_yaw < 0.01）提前标记 COLLAPSE，
这样即使 auto_localizer 因重试次数用尽而放弃，日志里也有记录。

================================================================
覆盖式写入，不占空间
================================================================
输出固定一个文件，启动时以 'w' 模式打开 → **每次运行覆盖上一次**。
默认 /tmp/fishbot_amcl_trace.csv（tmpfs，重启即清）。

另有软上限 MAX_ROWS，超过后停止写采样行但继续写事件行，
避免长时间挂着把磁盘写满。

================================================================
参数
================================================================
    output_path      输出文件（默认 /tmp/fishbot_amcl_trace.csv）
    sample_hz        采样频率（默认 5.0）
    heartbeat_sec    无变化时的心跳行间隔（默认 1.0）
    jump_dist        判定跳变的位置阈值（默认 0.15 m）
    jump_yaw_deg     判定跳变的朝向阈值（默认 5.0 度）
    max_rows         采样行软上限（默认 200000，约 20MB）
"""

import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

# ------------------------------------------------------------------
# 与 auto_localizer.py 保持一致的判据（改一处要同步另一处）
# ------------------------------------------------------------------
COLLAPSE_STD_XY = 0.005        # 低于此值判为粒子塌缩
COLLAPSE_STD_YAW = 0.01        # 弧度
CONVERGE_STD_XY = 0.08         # 高于此值判为未收敛
CONVERGE_STD_YAW = 0.15        # 弧度（约 8.6°）

CSV_HEADER = (
    't,wall,x,y,yaw,std_xy,std_yaw,'
    'mo_x,mo_y,mo_yaw,ob_x,ob_y,ob_yaw,'
    'd_pose,d_yaw,d_mo,d_ob,state,evt\n')


def _yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class AmclTracer(Node):
    """AMCL 重定位过程记录器。"""

    def __init__(self):
        super().__init__('amcl_tracer')

        self.declare_parameter('output_path', '/tmp/fishbot_amcl_trace.csv')
        self.declare_parameter('sample_hz', 5.0)
        self.declare_parameter('heartbeat_sec', 1.0)
        self.declare_parameter('jump_dist', 0.15)
        self.declare_parameter('jump_yaw_deg', 5.0)
        self.declare_parameter('max_rows', 200000)

        g = lambda k: self.get_parameter(k).value        # noqa: E731
        self.path = str(g('output_path'))
        self.period = 1.0 / max(0.5, float(g('sample_hz')))
        self.heartbeat = float(g('heartbeat_sec'))
        self.jump_dist = float(g('jump_dist'))
        self.jump_yaw = math.radians(float(g('jump_yaw_deg')))
        self.max_rows = int(g('max_rows'))

        # ---- 输入缓存 ----
        self.pose = None              # (x, y, yaw, std_xy, std_yaw, stamp)
        self.n_amcl = 0
        self.n_reseed = 0             # /initialpose 条数 = 重定位次数
        self.last_reseed = 0.0
        self.scan_hz = 0.0
        self.ekf_hz = 0.0
        self._scan_times = []
        self._ekf_times = []

        # ---- TF ----
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

        # ---- 上一采样点 ----
        self.prev = None              # (x, y, yaw, mo, ob)
        self.last_write = 0.0
        self.n_rows = 0
        self.n_events = 0
        self.jump_count = 0
        self.started = time.time()
        self._last_state = None
        self._capped = False

        # ---- 输出文件（覆盖式） ----
        try:
            os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
            self.fp = open(self.path, 'w', encoding='utf-8')
            self.fp.write('# fishbot AMCL trace  start=%s\n'
                          % time.strftime('%Y-%m-%d %H:%M:%S'))
            self.fp.write(CSV_HEADER)
            self.fp.flush()
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(
                '无法打开记录文件 %s: %s —— 追踪功能关闭' % (self.path, exc))
            self.fp = None

        # ---- 订阅（全部只读） ----
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        # auto_localizer 每次重新撒粒子都发这里 → 精确的重定位事件源
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.on_reseed, 10)
        # /scan 用 BEST_EFFORT：ydlidar 是 BEST_EFFORT 发布者，
        # 默认的 RELIABLE 订阅者与它不兼容，会一条也收不到
        # （在 tracer 里表现为 scan_hz 恒为 0，误判成「雷达没数据」）。
        self.create_subscription(
            LaserScan, '/scan', self.on_scan,
            QoSProfile(depth=5,
                       reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            Odometry, '/odometry/filtered', self.on_odom, 10)

        self.create_timer(self.period, self.tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('AMCL 重定位记录器已启动（只读）')
        self.get_logger().info('  输出    : %s（本次运行覆盖）' % self.path)
        self.get_logger().info('  采样    : %.1f Hz，无变化时 %.1fs 一行'
                               % (1.0 / self.period, self.heartbeat))
        self.get_logger().info('  判跳变  : >%.2fm 或 >%.0f°/采样'
                               % (self.jump_dist,
                                  math.degrees(self.jump_yaw)))
        self.get_logger().info('  塌缩判据: σxy<%.3f 且 σyaw<%.3frad'
                               % (COLLAPSE_STD_XY, COLLAPSE_STD_YAW))
        self.get_logger().info('=' * 60)

    # ================================================================
    # 回调
    # ================================================================
    def on_amcl(self, msg):
        p = msg.pose.pose
        c = msg.pose.covariance
        vx = max(c[0], 0.0)
        vy = max(c[7], 0.0)
        vyaw = max(c[35], 0.0)
        self.pose = (
            p.position.x, p.position.y, _yaw(p.orientation),
            math.sqrt(max(vx, vy)), math.sqrt(vyaw),
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.n_amcl += 1

    def on_reseed(self, msg):
        """auto_localizer 重新撒粒子 = 一次重定位。"""
        self.n_reseed += 1
        self.last_reseed = time.time()
        self._event('RESEED 第 %d 次重定位（重新撒粒子）' % self.n_reseed)

    def on_scan(self, msg):
        now = time.time()
        self._scan_times.append(now)
        self._scan_times = [t for t in self._scan_times if now - t < 3.0]
        if len(self._scan_times) > 1:
            span = self._scan_times[-1] - self._scan_times[0]
            if span > 0.5:
                self.scan_hz = (len(self._scan_times) - 1) / span

    def on_odom(self, msg):
        now = time.time()
        self._ekf_times.append(now)
        self._ekf_times = [t for t in self._ekf_times if now - t < 3.0]
        if len(self._ekf_times) > 1:
            span = self._ekf_times[-1] - self._ekf_times[0]
            if span > 0.5:
                self.ekf_hz = (len(self._ekf_times) - 1) / span

    # ================================================================
    # 采样
    # ================================================================
    def _tf(self, target, source):
        try:
            t = self.buf.lookup_transform(target, source, rclpy.time.Time())
        except Exception:                              # noqa: BLE001
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, _yaw(t.transform.rotation))

    def _state(self, std_xy, std_yaw):
        """镜像 auto_localizer 的判定，用于事后判读。"""
        if std_xy < COLLAPSE_STD_XY and std_yaw < COLLAPSE_STD_YAW:
            return 'COLLAPSE'
        if std_xy > CONVERGE_STD_XY or std_yaw > CONVERGE_STD_YAW:
            return 'DIVERGE'
        return 'CONVERGE'

    def tick(self):
        if self.fp is None:
            return

        now = time.time()
        if self.pose is None:
            # 没有 AMCL 输出也要留痕：这是「AMCL 没工作」的直接证据
            if now - self.last_write >= self.heartbeat:
                self.last_write = now
                self._row(None, None, None, None, 'NO_INPUT', '')
            return

        x, y, yaw, std_xy, std_yaw, _ = self.pose
        mo = self._tf('map', 'odom')
        ob = self._tf('odom', 'base_footprint')

        # ---- 变化量与跳变判定 ----
        d_pose = d_yaw = None
        d_mo = d_ob = None
        evt = ''
        if self.prev is not None:
            px, py, pa, pmo, pob = self.prev
            d_pose = math.hypot(x - px, y - py)
            d_yaw = abs(math.degrees(_wrap(yaw - pa)))
            if mo is not None and pmo is not None:
                d_mo = math.hypot(mo[0] - pmo[0], mo[1] - pmo[1])
            if ob is not None and pob is not None:
                d_ob = math.hypot(ob[0] - pob[0], ob[1] - pob[1])

            if d_pose > self.jump_dist or (d_yaw or 0) > math.degrees(self.jump_yaw):
                src = []
                if d_mo is not None and d_mo > self.jump_dist * 0.7:
                    src.append('AMCL')
                if d_ob is not None and d_ob > self.jump_dist * 0.7:
                    src.append('EKF')
                tag = '+'.join(src) or '未知'
                self.jump_count += 1
                evt = 'JUMP[%s] %.2fm/%.1f°' % (tag, d_pose, d_yaw or 0)
                self._event('跳变 #%d  %s  来源=%s  (map→odom=%s, odom→base=%s)'
                            % (self.jump_count, evt,
                               tag,
                               '%.2f' % d_mo if d_mo is not None else '缺失',
                               '%.2f' % d_ob if d_ob is not None else '缺失'))

        self.prev = (x, y, yaw, mo, ob)

        state = self._state(std_xy, std_yaw)

        # 状态跃迁单独记一笔（塌缩恢复最关键）
        if state != self._last_state:
            if self._last_state is not None:
                evt = (evt + ' ' if evt else '') + \
                      'STATE %s→%s' % (self._last_state, state)
                self._event('状态 %s → %s  (σxy=%.4f σyaw=%.4frad)'
                            % (self._last_state, state, std_xy, std_yaw))
            self._last_state = state

        # ---- 是否需要写一行 ----
        # 有跳变/事件 → 必写；否则按心跳间隔写
        if evt or (now - self.last_write >= self.heartbeat):
            self.last_write = now
            self._row((x, y, yaw, std_xy, std_yaw),
                      mo, ob,
                      (d_pose, d_yaw, d_mo, d_ob), state, evt)

    # ================================================================
    # 写文件
    # ================================================================
    def _row(self, pose, mo, ob, deltas, state, evt):
        if self.fp is None or self._capped:
            return
        if self.n_rows >= self.max_rows:
            self._capped = True
            self.fp.write('# 达到 max_rows=%d，停止写采样行'
                          '（事件行继续写）\n' % self.max_rows)
            self.fp.flush()
            return

        # ★ 用列表 + join 拼 CSV，不要用长格式字符串。
        #   之前手写 18 个占位符对 19 个字段，少一个就抛
        #   TypeError: not all arguments converted during string
        #   formatting —— 而且异常在定时器回调里，被 executor 吞成
        #   「节点静默退出」，排查了很久。列表拼接从根上避免数错。
        def f3(v):
            return '' if v is None else '%.3f' % v

        def f4(v):
            return '' if v is None else '%.4f' % v

        def deg(v):
            return '' if v is None else '%.2f' % math.degrees(v)

        if pose is None:
            x = y = yaw = sxy = syaw = None
        else:
            x, y, yaw, sxy, syaw = pose

        dp = dy = dmo = dob = None
        if deltas is not None:
            dp, dy, dmo, dob = deltas

        cells = [
            '%.2f' % (time.time() - self.started),
            time.strftime('%H:%M:%S'),
            f3(x), f3(y), deg(yaw), f4(sxy), f4(syaw),
            f3(mo[0]) if mo else '', f3(mo[1]) if mo else '',
            deg(mo[2]) if mo else '',
            f3(ob[0]) if ob else '', f3(ob[1]) if ob else '',
            deg(ob[2]) if ob else '',
            f3(dp), '' if dy is None else '%.2f' % dy, f3(dmo), f3(dob),
            state, evt,
        ]

        try:
            self.fp.write(','.join(cells) + '\n')
            self.fp.flush()
            self.n_rows += 1
        except Exception:                              # noqa: BLE001
            self.fp = None

    def _event(self, text):
        """事件行以 # 开头，便于 grep '^#' 快速看时间线。"""
        self.n_events += 1
        if self.fp is None:
            return
        try:
            self.fp.write('# %s  %s\n' % (time.strftime('%H:%M:%S'), text))
            self.fp.flush()
        except Exception:                              # noqa: BLE001
            self.fp = None

    # ================================================================
    def finish(self):
        """退出时写总结并打印。"""
        elapsed = time.time() - self.started
        summary = (
            '采样行=%d  事件=%d  跳变=%d  重定位=%d  AMCL消息=%d  '
            '最后scan=%.1fHz 最后EKF=%.1fHz'
            % (self.n_rows, self.n_events, self.jump_count,
               self.n_reseed, self.n_amcl, self.scan_hz, self.ekf_hz))
        if self.fp is not None:
            try:
                self.fp.write('# ---- 结束：运行 %.0fs  %s ----\n'
                              % (elapsed, summary))
                self.fp.flush()
                self.fp.close()
            except Exception:                          # noqa: BLE001
                pass
            self.fp = None
        self.get_logger().info('AMCL 记录结束：运行 %.0fs  %s'
                               % (elapsed, summary))
        self.get_logger().info('  文件: %s' % self.path)


def main(args=None):
    rclpy.init(args=args)
    node = AmclTracer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:                                  # noqa: BLE001
        # 不要把异常吞掉：之前吞掉导致「节点 3 秒后静默退出」查不出原因。
        import traceback
        traceback.print_exc()
    finally:
        try:
            node.finish()
        except Exception:                              # noqa: BLE001
            import traceback
            traceback.print_exc()
        try:
            node.destroy_node()
        except Exception:                              # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
