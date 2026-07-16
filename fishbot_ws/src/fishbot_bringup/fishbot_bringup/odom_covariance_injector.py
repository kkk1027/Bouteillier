#!/usr/bin/env python3
"""
协方差注入节点 —— 给 ESP32 的 /odom 补上合理协方差，再转发给 EKF。

用法（bringup 中自动包含）：
    ros2 run fishbot_bringup odom_covariance_injector.py

数据流：
    ESP32 /odom (cov=0) → 本节点 → /odom_ekf (cov=合理值) → EKF

为什么需要：
    ESP32 用 micro-ROS 发布 /odom 时协方差全为零，
    EKF 读到零协方差会认为传感器完美，导致自身协方差也收敛到零。
    注入合理协方差让 EKF 能正确估计不确定性。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry


class OdomCovarianceInjector(Node):
    def __init__(self):
        super().__init__('odom_covariance_injector')

        # 声明可调参数
        self.declare_parameter('pose_cov_xy', 0.01)      # x,y 位置方差
        self.declare_parameter('pose_cov_yaw', 0.005)     # yaw 朝向方差
        self.declare_parameter('twist_cov_vx', 0.02)      # vx 线速度方差
        self.declare_parameter('twist_cov_vyaw', 0.01)    # vyaw 角速度方差

        # ESP32 用 BEST_EFFORT 发布，订阅端必须匹配
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub = self.create_subscription(
            Odometry, '/odom', self.callback, qos)

        self.pub = self.create_publisher(
            Odometry, '/odom_ekf', 10)

        self.get_logger().info(
            '协方差注入节点已启动: /odom → /odom_ekf (带协方差)')

    def callback(self, msg: Odometry):
        p_xy = self.get_parameter('pose_cov_xy').value
        p_yaw = self.get_parameter('pose_cov_yaw').value
        t_vx = self.get_parameter('twist_cov_vx').value
        t_vyaw = self.get_parameter('twist_cov_vyaw').value

        # 注入 pose 协方差 (36 = 6x6 矩阵)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = p_xy    # x 方差
        msg.pose.covariance[7] = p_xy    # y 方差
        msg.pose.covariance[14] = 1e-9   # z (2D 模式不用)
        msg.pose.covariance[21] = 1e-9   # roll
        msg.pose.covariance[28] = 1e-9   # pitch
        msg.pose.covariance[35] = p_yaw  # yaw 方差

        # 注入 twist 协方差
        msg.twist.covariance = [0.0] * 36
        msg.twist.covariance[0] = t_vx     # vx 方差
        msg.twist.covariance[7] = 1e-9     # vy
        msg.twist.covariance[14] = 1e-9    # vz
        msg.twist.covariance[21] = 1e-9    # vroll
        msg.twist.covariance[28] = 1e-9    # vpitch
        msg.twist.covariance[35] = t_vyaw  # vyaw 方差

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = OdomCovarianceInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
