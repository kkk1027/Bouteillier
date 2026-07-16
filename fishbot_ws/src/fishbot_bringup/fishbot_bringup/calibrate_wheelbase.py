#!/usr/bin/env python3
"""
FishBot 轮距（Wheelbase）标定工具

差速机器人如果 wheel_distance 参数不准，yaw 会随着转动线性漂移。
本脚本通过"原地旋转 N 圈 → 对比 odom 报告角度 vs 实际物理角度"
来自动计算正确的 wheelbase 值。

原理:
    真实角速度 = (vr - vl) / wheelbase_true
    odom角速度 = (vr - vl) / wheelbase_current

    旋转一圈: odom_angle / actual_angle = wheelbase_true / wheelbase_current
    → wheelbase_true = wheelbase_current × (odom_angle / actual_angle)

使用步骤:
    1. 启动机器人 (bringup + micro_ros_agent)
    2. 把机器人放在平整地面上，用地板胶带标记其正前方朝向
    3. 运行本脚本:
       ros2 run fishbot_bringup calibrate_wheelbase    (默认顺时针 10 圈)
       ros2 run fishbot_bringup calibrate_wheelbase --turns 5 --direction ccw
    4. 机器人旋转停止后，目测或用尺子量实际转了多少圈
    5. 输入实际角度（如 3600 表示正好 10 圈，3420 表示只转了 9.5 圈）
    6. 脚本输出修正后的 wheelbase 值
    7. 将这个值写入 ESP32 固件 main.cpp 的 kinematics.set_wheel_distance()
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def unwrap_angle_diff(new_angle: float, prev_angle: float) -> tuple[float, float]:
    """
    角度展开：让 yaw 可以超越 [-π, π] 范围连续增长。
    返回 (累积角度, 当前展开角度)
    """
    diff = new_angle - prev_angle
    # 规范化到 [-π, π]
    diff = math.atan2(math.sin(diff), math.cos(diff))
    return diff


class WheelbaseCalibrator(Node):
    def __init__(self, turns: int, direction: str, current_wheelbase: float):
        super().__init__('wheelbase_calibrator')

        self.turns = turns
        self.direction = 1.0 if direction == 'ccw' else -1.0  # ROS: z-up, ccw=+
        self.current_wheelbase = current_wheelbase  # mm

        # 计算目标角度 (弧度)
        self.target_angle = turns * 2.0 * math.pi

        # 旋转参数
        self.angular_speed = 0.4  # rad/s, 慢速避免打滑
        self.rotate_duration = self.target_angle / self.angular_speed

        # 状态变量
        self.state = 'RECORD_INIT'
        # ESP32 用 BEST_EFFORT 发布 /odom，订阅端必须匹配
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.yaw_samples = []  # 记录初始 yaw
        self.init_yaw = None
        self.init_samples_needed = 30  # 静止时采集 30 帧求平均
        self.final_yaw = None
        self.cumulative_angle = 0.0
        self.last_yaw = None
        self.rotation_start_time = None

        self.get_logger().info(f'=== FishBot Wheelbase 标定 ===')
        self.get_logger().info(f'当前 wheelbase: {current_wheelbase:.1f} mm')
        self.get_logger().info(f'旋转圈数: {turns} 圈 ({direction})')
        self.get_logger().info(f'目标角度: {math.degrees(self.target_angle):.1f}°')
        self.get_logger().info(f'旋转速度: {self.angular_speed:.2f} rad/s')
        self.get_logger().info(f'预计耗时: {self.rotate_duration:.1f} 秒')
        self.get_logger().info('')
        self.get_logger().info('采集初始朝向...（请勿移动机器人）')

        # 主循环定时器
        self.timer = self.create_timer(0.05, self.main_loop)

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # 从四元数提取 yaw
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        if self.state == 'RECORD_INIT':
            self.yaw_samples.append(yaw)
        elif self.state == 'ROTATING':
            if self.last_yaw is not None:
                diff = unwrap_angle_diff(yaw, self.last_yaw)
                self.cumulative_angle += diff
            self.last_yaw = yaw
        elif self.state == 'RECORD_FINAL':
            self.final_yaw = yaw

    def main_loop(self):
        if self.state == 'RECORD_INIT':
            if len(self.yaw_samples) >= self.init_samples_needed:
                self.init_yaw = sum(self.yaw_samples) / len(self.yaw_samples)
                self.last_yaw = self.init_yaw
                self.get_logger().info(
                    f'初始朝向: {math.degrees(self.init_yaw):.2f}°')
                self.get_logger().info('开始旋转...')
                self.state = 'ROTATING'
                self.rotation_start_time = time.time()
                # 发送旋转指令
                twist = Twist()
                twist.angular.z = self.direction * self.angular_speed
                self.cmd_pub.publish(twist)

        elif self.state == 'ROTATING':
            elapsed = time.time() - self.rotation_start_time
            # 每 2 秒打印进度
            if int(elapsed) % 2 == 0 and int(elapsed) > 0:
                progress = abs(self.cumulative_angle) / self.target_angle * 100
                self.get_logger().info(
                    f'旋转中... {progress:.0f}% '
                    f'({math.degrees(self.cumulative_angle):.0f}° / '
                    f'{math.degrees(self.target_angle):.0f}°)')
            if elapsed >= self.rotate_duration:
                # 停止
                self.cmd_pub.publish(Twist())  # 全零停止
                self.get_logger().info('旋转完成，停止电机...')
                time.sleep(0.5)  # 等轮子停下来
                self.state = 'RECORD_FINAL'

        elif self.state == 'RECORD_FINAL':
            odom_reported_deg = math.degrees(self.cumulative_angle)
            self.get_logger().info('')
            self.get_logger().info('=' * 55)
            self.get_logger().info(f'Odom 报告的旋转角度: {odom_reported_deg:+.1f}°')
            self.get_logger().info(f'  ({abs(self.cumulative_angle) / (2*math.pi):.2f} 圈)')
            self.get_logger().info('')
            self.get_logger().info('现在请观察机器人实际转了多少圈，输入实际角度：')
            self.get_logger().info('  例: 3600  = 正好 10 圈')
            self.get_logger().info('  例: 3420  = 转了 9.5 圈（9.5×360=3420）')
            self.get_logger().info('  例: 3780  = 转了 10.5 圈')
            self.destroy_timer(self.timer)

            # 从终端读取用户输入
            try:
                actual_deg = float(
                    input(f'\n输入实际旋转角度 (度): ').strip())
            except (EOFError, ValueError):
                self.get_logger().error('无效输入，使用 odom 报告值（无修正）')
                actual_deg = odom_reported_deg

            # 计算修正
            if abs(actual_deg) < 1.0:
                self.get_logger().error('角度太小，无法标定')
                rclpy.shutdown()
                return

            corrected_wheelbase = (
                self.current_wheelbase * odom_reported_deg / actual_deg)

            error_pct = (corrected_wheelbase - self.current_wheelbase
                         ) / self.current_wheelbase * 100

            self.get_logger().info('')
            self.get_logger().info('=' * 55)
            self.get_logger().info('           标 定 结 果')
            self.get_logger().info('=' * 55)
            self.get_logger().info(
                f'  当前 wheelbase:  {self.current_wheelbase:6.1f} mm')
            self.get_logger().info(
                f'  修正 wheelbase:  {corrected_wheelbase:6.1f} mm')
            self.get_logger().info(
                f'  误差:             {error_pct:+.1f}%')
            self.get_logger().info(
                f'  Odom 报告:        {odom_reported_deg:+.1f}°')
            self.get_logger().info(
                f'  实际旋转:         {actual_deg:+.1f}°')
            self.get_logger().info('')
            self.get_logger().info(
                '请将 ESP32 固件中的这行代码修改为:')
            self.get_logger().info(
                f'    kinematics.set_wheel_distance({corrected_wheelbase:.1f});')
            self.get_logger().info('')
            self.get_logger().info(
                '文件位置: fishbot_motion_control/src/main.cpp')
            self.get_logger().info('=' * 55)

            # 可选：再做一次反向验证
            self.get_logger().info('')
            response = input('是否反向旋转验证？(y/n): ').strip().lower()
            if response == 'y':
                self.get_logger().info(
                    f'建议改完 wheelbase={corrected_wheelbase:.1f} 后重新运行本脚本验证')

            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='FishBot 轮距标定工具 - 通过原地旋转对比 odom vs 实际角度来标定 wheelbase')
    parser.add_argument('--turns', type=int, default=10,
                        help='旋转圈数 (默认: 10, 建议 >=5)')
    parser.add_argument('--direction', choices=['cw', 'ccw'],
                        default='cw', help='旋转方向 cw=顺时针 ccw=逆时针 (默认: cw)')
    parser.add_argument('--wheelbase', type=float, default=175.0,
                        help='当前固件中的 wheelbase 值, 单位 mm (默认: 175)')
    args = parser.parse_args()

    if args.turns < 1:
        print('错误: 圈数必须 >= 1')
        sys.exit(1)

    rclpy.init()
    node = WheelbaseCalibrator(args.turns, args.direction, args.wheelbase)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())
        print('\n标定已取消')
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
