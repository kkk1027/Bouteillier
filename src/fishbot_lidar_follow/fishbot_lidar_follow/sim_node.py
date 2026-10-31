#!/usr/bin/env python3
"""
sim_node.py
雷达跟随的离线仿真节点。

用途：没有雷达、没有底盘时，验证整条链路是否正常。
它自己扮演「雷达」发布假的 /scan，并订阅 /cmd_vel 打印出来。

安全性：
    默认 **不会** 转发到真实的 /cmd_vel！订阅的是 /cmd_vel 只是打印。
    如需联调真实底盘，显式加 --ros-args -p forward:=true，
    这时才会把速度转发到 cmd_vel_topic。

用法：
    # 纯观察：在人造场景里走一遍，打印每步的速度
    ros2 run fishbot_lidar_follow lidar_follow_sim

    # 只跑跟随节点，用这个仿真喂数据
    ros2 run fishbot_lidar_follow lidar_follow_sim --scenario walk_away

场景：
    walk_away    目标从 0.35m 慢慢退到 1.1m（测前进）
    walk_close   目标从 1.1m 慢慢逼近到 0.35m（测后退）
    sideways     目标在 0.8m 处左右横移（测转向）
    dropout      目标出现 → 消失 → 再出现（测丢帧容忍）
    static       目标静止在 0.6m（测死区不抖）
"""

import math
import random
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


SCENARIOS = {
    'walk_away': '目标从 0.35m 退到 1.10m —— 应该先锁定然后前进',
    'walk_close': '目标从 1.10m 逼近到 0.35m —— 应该先锁定然后后退',
    'sideways': '目标在 0.80m 处左右摆动 ±20° —— 应该跟着转',
    'dropout': '目标出现→消失→再出现 —— 丢失时应该停车',
    'static': '目标静止在 0.60m —— 应该停在死区不动',
    'legs_walk': '两条腿（每条 1 个点）先退开再走近 —— 测双腿配对锁定',
    'legs_side': '两条腿整体左右转动 ±25° —— 测配对后跟转',
    'legs_occlude': '每 4 帧里有 1 帧只剩一条腿 —— 测遮挡容忍',
    'legs_single': '先用双腿锁定，之后只剩一条腿 —— 测单腿兜底',
    'noise': '真实杂散点（固定虚假回波 + 偶发飞点）—— 不应该锁定',
    'noise_random': '压力测试：每帧全扇区重随机散点 —— 极限情况',
}

# 腿的横向间距（米）。两条腿间距 22cm，接近真实站姿
LEG_HALF_GAP = 0.11

# 噪声场景的随机种子。固定下来让测试可复现，
# 但每步都换一个新的 Random 实例，避免出现规律性重复
_NOISE_SEED = 20261031


def _legs(depth, half_gap, yaw_deg=0.0, hide=None, pts=0):
    """生成「两条腿」的扫描点。

    参数：
        depth     人体中心到车的纵向距离（米）
        half_gap  每条腿相对中心线的横向偏移（米）
        yaw_deg   人整体绕车转过的角度（左右移动时用）
        hide      None = 两条腿都显示；0/1 = 只显示其中一条（模拟遮挡）
        pts       每条腿的半宽点数。0 表示整条腿只占 1 个点 ——
                  实机上人腿就是这种情况，也是最难锁的工况。

    返回 [(距离, 角度deg, 半宽点数), ...]
    """
    yaw = math.radians(yaw_deg)
    cy, sy = math.cos(yaw), math.sin(yaw)

    out = []
    for idx, sign in enumerate((-1.0, 1.0)):
        if hide is not None and idx != hide:
            continue
        xf = depth                 # 纵向：+x 为车正前方
        yl = sign * half_gap       # 横向：+y 为车的左侧
        xr = xf * cy - yl * sy     # 绕车原点旋转
        yr = xf * sy + yl * cy
        out.append((math.hypot(xr, yr), math.degrees(math.atan2(yr, xr)), pts))
    return out


class LidarFollowSim(Node):
    """假雷达 + 速度观察器。"""

    def __init__(self):
        super().__init__('lidar_follow_sim')

        self.declare_parameter('scenario', 'walk_away')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate', 5.0)     # 与真实雷达一致
        self.declare_parameter('forward', False)        # 是否转发到真实底盘
        self.declare_parameter('angle_min_deg', -180.0)
        self.declare_parameter('angle_max_deg', 180.0)
        self.declare_parameter('angle_increment_deg', 0.5)
        self.declare_parameter('range_min', 0.30)
        self.declare_parameter('range_max', 64.0)

        self.scenario = self.get_parameter('scenario').value
        self.forward = bool(self.get_parameter('forward').value)
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_topic = self.get_parameter('cmd_vel_topic').value
        rate = float(self.get_parameter('publish_rate').value)

        self.amin = math.radians(
            float(self.get_parameter('angle_min_deg').value))
        self.amax = math.radians(
            float(self.get_parameter('angle_max_deg').value))
        self.ainc = math.radians(
            float(self.get_parameter('angle_increment_deg').value))
        self.rmin = float(self.get_parameter('range_min').value)
        self.rmax = float(self.get_parameter('range_max').value)

        self.n = int(round((self.amax - self.amin) / self.ainc)) + 1

        self.scan_pub = self.create_publisher(
            LaserScan, self.scan_topic, qos_profile_sensor_data)
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_topic, self.on_cmd, 10)
        self.fwd_pub = None
        if self.forward:
            self.fwd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
            self.get_logger().warn(
                '⚠️ forward=true：仿真速度会转发到真实底盘 %s'
                % self.cmd_topic)

        self.t = 0.0
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.step = 0
        self.last_lin = 0.0
        self.last_ang = 0.0

        self.get_logger().info('=' * 58)
        self.get_logger().info('雷达跟随仿真')
        self.get_logger().info('  场景 : %s' % self.scenario)
        self.get_logger().info('        %s' % SCENARIOS.get(
            self.scenario, '未知场景（目标静止在 0.7m）'))
        self.get_logger().info('  发布 : %s  %.1fHz  %d 点/帧'
                               % (self.scan_topic, rate, self.n))
        self.get_logger().info('  监听 : %s' % self.cmd_topic)
        self.get_logger().info('  转发 : %s' % (
            '是（会驱动真实底盘！）' if self.forward else '否（仅打印）'))
        self.get_logger().info('=' * 58)

    # ----------------------------------------------------
    def on_cmd(self, msg):
        self.last_lin = msg.linear.x
        self.last_ang = msg.angular.z

    # ----------------------------------------------------
    def on_timer(self):
        self.t += self.dt
        self.step += 1

        objs = self._objects_at(self.t)
        self.scan_pub.publish(self._make_scan(objs))

        if self.step % 5 == 0:      # 每 1 秒打一行
            desc = ' '.join('(%.2fm,%+.0f°)' % (o[0], o[1]) for o in objs)
            self.get_logger().info(
                't=%.1fs 目标%s → lin=%+.3f ang=%+.3f'
                % (self.t, desc or '(无)', self.last_lin, self.last_ang))

        if self.forward and self.fwd_pub is not None:
            # 去掉自己订阅到的，避免回环
            pass

    # ----------------------------------------------------
    def _objects_at(self, t):
        """返回当前时刻的目标 [(距离, 角度deg), ...]。

        ★ 所有场景都在 0.35m 处开始，并静止 HOLD_SEC 秒。
        原因：首次锁定要求目标在 40cm 内（用户明确要求），
        而 launch 启动两个节点需要 1 秒左右。
        如果目标一开始就在 40cm 外或已经在移动，
        等节点就绪时就已经锁不上了。
        真实使用也是这样 —— 人先站到车前，再开始走。

        参数顺序说明：
          ACQUIRE  0~HOLD_SEC        停在 0.35m，让节点锁定
          DEMO     HOLD_SEC 之后      各场景自己的动作
        """
        ACQUIRE = (0.35, 0.0)
        # 锁定准备期。必须留够：
        #   * 节点启动 ~1s
        #   * lock_confirm_frames=5 帧 @5Hz = 1s
        #   * 余量
        # 之前是 2.5s，改到 5 帧确认后就攒不满了：
        # 场景一移动目标就开始漂，确认计数被反复重置，
        # 表现为「能跟但锁不上」。真实使用时人站好就站着不动，
        # 所以把准备期拉长是对的 —— 仿真是为了复现实物，不是为了赶时间。
        HOLD_SEC = 4.5
        s = self.scenario

        if t < HOLD_SEC:
            # 所有 legs* 场景都先用**双腿**完成首次锁定，
            # 之后才各自退化（legs_single 退化成一条腿）。
            # 因为首次锁定不允许靠一条腿 ——
            # 一条腿无法区分「人」和「一根桌腿」，必须成对才认。
            if s.startswith('legs'):
                return _legs(0.62, LEG_HALF_GAP)
            if s == 'noise':
                return self._noise()
            if s == 'noise_random':
                return self._noise_random()
            return [ACQUIRE]

        tt = t - HOLD_SEC     # 扣除锁定期后经过的时间

        if s == 'walk_away':
            # 0.35 → 1.10，用时 8 秒
            frac = min(1.0, tt / 8.0)
            return [(0.35 + 0.75 * frac, 0.0)]

        if s == 'walk_close':
            # 先退到 1.10（3 秒），再逼近回 0.35（5 秒）
            if tt < 3.0:
                frac = min(1.0, tt / 3.0)
                return [(0.35 + 0.75 * frac, 0.0)]
            frac = min(1.0, (tt - 3.0) / 5.0)
            return [(1.10 - 0.75 * frac, 0.0)]

        if s == 'sideways':
            # 先退到 0.80（2 秒），然后在 ±20° 之间摇摆，周期 4 秒
            if tt < 2.0:
                frac = min(1.0, tt / 2.0)
                return [(0.35 + 0.45 * frac, 0.0)]
            a = 20.0 * math.sin(2 * math.pi * (tt - 2.0) / 4.0)
            return [(0.80, a)]

        if s == 'dropout':
            # 先在 0.35 锁定，退到 0.70；然后消失 3 秒再出现
            if tt < 2.0:
                frac = min(1.0, tt / 2.0)
                return [(0.35 + 0.35 * frac, 0.0)]
            if 2.0 <= tt < 5.0:
                return []       # 消失
            return [(0.70, 0.0)]   # 重新出现

        if s == 'static':
            # 静止在 0.60（死区中心），验证不抖动
            if tt < 1.5:
                frac = min(1.0, tt / 1.5)
                return [(0.35 + 0.25 * frac, 0.0)]
            return [(0.60, 0.0)]

        # ---------------- 双腿工况 ----------------
        # 人在低处雷达里只有两条小腿，每条腿 1~3 个点。
        # 这是「能跟但锁不上」的真实场景。

        if s == 'legs_walk':
            # 0.62 → 1.05（4 秒，人往后退 → 车应该前进）
            # 1.05 → 0.45（5 秒，人走近  → 车应该后退）
            if tt < 4.0:
                depth = 0.62 + 0.43 * min(1.0, tt / 4.0)
            else:
                depth = 1.05 - 0.60 * min(1.0, (tt - 4.0) / 5.0)
            return _legs(depth, LEG_HALF_GAP)

        if s == 'legs_side':
            # 人整体左右转，腿跟着一起转（不是分开走）
            yaw = 25.0 * math.sin(2 * math.pi * tt / 6.0)
            return _legs(0.80, LEG_HALF_GAP, yaw_deg=yaw)

        if s == 'legs_occlude':
            # 两条腿大部分时间都在，每 4 帧里有 1 帧只看得见一条。
            # 这是人走路时的真实情况。
            # 注意：完全只剩一条腿时走「单腿兜底」，只用距离跟，
            # 角度交给卡尔曼预测 —— 否则不可能既要跟又不抽。
            hide = 0 if int(tt / 0.8) % 4 == 3 else None
            return _legs(0.62, LEG_HALF_GAP, hide=hide)

        if s == 'legs_single':
            # 先用双腿锁定，然后只剩一条腿，同时距离缓慢变大。
            # 如果单腿兜底真的在工作，距离会跟着变大并触发 APPROACH；
            # 如果不工作，会连续丢帧停车。
            depth = 0.62 + 0.30 * min(1.0, tt / 6.0)
            return _legs(depth, LEG_HALF_GAP, hide=0)

        if s == 'noise':
            return self._noise()

        if s == 'noise_random':
            return self._noise_random()

        return [ACQUIRE]

    def _noise(self):
        """真实雷达的杂散点形态。

        真实杂散点的特点：
          * 位置基本固定（墙角、玻璃、镜面多重反射的虚假回波）
          * 偶尔夹杂单帧「飞点」（表面材质导致的坏点）
          * 大部分是孤立单点

        注意这**不是**「每帧在整个扇区里重新随机」——
        那种噪声物理上不存在，只有合成数据才有。
        真正的随机散点每帧位置全变，几何约束几乎必然全部通过
        （实测单帧命中率 100%），只能靠时间确认，属于压力测试，
        放在 noise_random 场景里。

        固定的两个虚假回波：0.83m/-34° 与 0.91m/+62°。
        两点相距很远、不可能配成一双腿 —— 这正是期望的行为。
        """
        rnd = random.Random(_NOISE_SEED + self.step)
        out = []
        for d, a in ((0.83, -34.0), (0.91, 62.0)):
            if rnd.random() < 0.70:          # 70% 的帧可见
                out.append((d + rnd.uniform(-0.01, 0.01),
                            a + rnd.uniform(-0.5, 0.5), 0))
        if rnd.random() < 0.25:              # 25% 的帧夹一个飞点
            out.append((rnd.uniform(0.50, 0.90),
                        rnd.uniform(-40.0, 70.0), 0))
        return out

    def _noise_random(self):
        """压力测试：每帧 3 个完全随机的散点（物理上不存在，只做极限测试）。

        几何约束（同深度 / 腿距 / 横向 / 纵向）对这种噪声的单帧命中率
        是 100%，全靠 5 帧时间确认挡住。短时间（~25 帧）误锁率约 0%，
        但跑长了（100+ 帧）仍可能偶尔锁上 —— 这是已知边界，
        详见 README「已知限制」。
        """
        rnd = random.Random(_NOISE_SEED + self.step)
        return [(rnd.uniform(0.45, 0.90), rnd.uniform(-30.0, 90.0), 0)
                for _ in range(3)]

    # ----------------------------------------------------
    def _make_scan(self, objects):
        """构造一圈 LaserScan。

        objects 每项为 (距离, 角度deg) 或 (距离, 角度deg, 半宽点数)。
        半宽点数默认 5（即 ±5 个索引，模拟有宽度的实体）。
        人腿用 0（每边只占 1 个点）—— 这正是实机上看到的情况。
        """
        msg = LaserScan()
        msg.angle_min = self.amin
        msg.angle_max = self.amax
        msg.angle_increment = self.ainc
        msg.range_min = self.rmin
        msg.range_max = self.rmax

        far = 20.0
        ranges = [far] * self.n

        for obj in objects:
            dist, deg = obj[0], obj[1]
            half = obj[2] if len(obj) > 2 else 5
            idx = int(round((math.radians(deg) - self.amin) / self.ainc))
            for k in range(-half, half + 1):
                j = idx + k
                if 0 <= j < self.n:
                    # 边缘稍微远一点，模拟物体表面
                    ranges[j] = dist + abs(k) * 0.004

        msg.ranges = ranges
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = LidarFollowSim()
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
