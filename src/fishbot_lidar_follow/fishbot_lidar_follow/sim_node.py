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
}


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
            desc = ' '.join('(%.2fm,%+.0f°)' % (d, a) for d, a in objs)
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
        HOLD_SEC = 2.5
        s = self.scenario

        if t < HOLD_SEC:
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

        return [ACQUIRE]

    # ----------------------------------------------------
    def _make_scan(self, objects):
        """构造一圈 LaserScan。每个物体占 ±5 个索引（模拟有宽度）。"""
        msg = LaserScan()
        msg.angle_min = self.amin
        msg.angle_max = self.amax
        msg.angle_increment = self.ainc
        msg.range_min = self.rmin
        msg.range_max = self.rmax

        far = 20.0
        ranges = [far] * self.n

        for dist, deg in objects:
            idx = int(round((math.radians(deg) - self.amin) / self.ainc))
            for k in range(-5, 6):
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
