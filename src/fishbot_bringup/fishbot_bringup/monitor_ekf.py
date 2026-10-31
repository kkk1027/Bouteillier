#!/usr/bin/env python3
"""
EKF 滤波监控工具 —— 实时显示 /odometry/filtered 的 yaw 和协方差收敛状态。

用法:
    ros2 run fishbot_bringup monitor_ekf.py
    ros2 run fishbot_bringup monitor_ekf.py --simple   # 简洁模式

判断标准:
    1. 静止时 yaw 波动 < 0.01 rad/s 持续 5s → yaw 不漂
    2. pose.covariance[35] (yaw 方差) 随时间下降并稳定在 < 0.01 → 协方差收敛
    3. twist.covariance[0] (vx 方差) 稳定在 < 0.01 → 速度估计收敛
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
import math
import time
import argparse
import os


def quat_to_yaw(q):
    """四元数转 yaw 角 [-pi, pi]"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class EKFMonitor(Node):
    def __init__(self, simple=False):
        super().__init__('ekf_monitor')
        self.simple = simple
        self.last_yaw = None
        self.last_time = None
        self.last_print_time = time.time()

        # 统计
        self.start_time = time.time()
        self.max_yaw_rate_while_still = 0.0
        self.sample_count = 0
        self.cov_stable_count = 0

        # 最新协方差
        self.pose_cov_yaw = None   # pose.covariance[35]
        self.twist_cov_vx = None   # twist.covariance[0]

        self.sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        if not self.simple:
            self.print_header()

    def print_header(self):
        hdr = (
            "\n"
            + "=" * 82 + "\n"
            + "  EKF 滤波监控 — 判断标准\n"
            + "  ✅ 协方差收敛:  pose.cov[35](yaw方差) 下降并稳定在 < 0.01\n"
            + "  ✅ yaw 不漂移:  静止时角速度 < 0.01 rad/s\n"
            + "  ✅ 速度收敛:    twist.cov[0](vx方差) 稳定在 < 0.01\n"
            + "=" * 82
        )
        self.get_logger().info(hdr)
        self.get_logger().info(
            f"{'Time':>8s}  {'Yaw(deg)':>8s}  {'YawRate':>8s}  "
            f"{'PoseCov[35]':>12s}  {'TwistCov[0]':>12s}  {'Status':s}"
        )
        self.get_logger().info("-" * 82)

    def callback(self, msg: Odometry):
        now = time.time()
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        vyaw = msg.twist.twist.angular.z           # 机器人自身角速度
        vx = msg.twist.twist.linear.x               # 机器人自身线速度
        self.pose_cov_yaw = msg.pose.covariance[35]
        self.twist_cov_vx = msg.twist.covariance[0]
        self.sample_count += 1

        # 计算 yaw rate (heading 变化率)
        yaw_rate = 0.0
        if self.last_yaw is not None and self.last_time is not None:
            dt = now - self.last_time
            if dt > 0.001:
                dyaw = yaw - self.last_yaw
                dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
                yaw_rate = dyaw / dt

        self.last_yaw = yaw
        self.last_time = now

        # 每秒打印一次
        if now - self.last_print_time < 1.0:
            return
        self.last_print_time = now

        # --- 判断是否正在主动运动 ---
        # 如果 twist 角速度或线速度显著，说明机器人在主动运动，不判定为漂移
        is_moving = abs(vyaw) > 0.02 or abs(vx) > 0.02

        # --- 判断状态 ---
        cov_ok = self.pose_cov_yaw is not None and self.pose_cov_yaw > 1e-9 and self.pose_cov_yaw < 0.01
        twist_ok = self.twist_cov_vx is not None and self.twist_cov_vx > 1e-9 and self.twist_cov_vx < 0.01

        # yaw 漂移判断：只在机器人静止时检查
        yaw_ok = True
        if is_moving:
            yaw_status = '🔄MOVING'
        elif abs(yaw_rate) < 0.01:
            yaw_status = '✅STILL'
        else:
            yaw_ok = False
            yaw_status = '❌DRIFT'

        if cov_ok:
            self.cov_stable_count += 1

        if self.simple:
            status = "✅" if (cov_ok and yaw_ok and twist_ok) else "⚠️ "
            if not cov_ok and self.pose_cov_yaw is not None:
                if self.pose_cov_yaw < 1e-9:
                    status = "❌ COV=0"
                elif self.pose_cov_yaw > 0.1:
                    status = "❌ COV>0.1"
                else:
                    status = "⏳ COV"
            self.get_logger().info(
                f"yaw={math.degrees(yaw):6.1f}°  "
                f"yaw_rate={yaw_rate:+.4f}  "
                f"cov_yaw={self.pose_cov_yaw:.4f}  "
                f"cov_vx={self.twist_cov_vx:.4f}  "
                f"{status}"
            )
        else:
            status_parts = []
            if cov_ok:
                status_parts.append("✅COV")
            elif self.pose_cov_yaw is not None and self.pose_cov_yaw < 1e-9:
                status_parts.append("❌COV=0")
            elif self.pose_cov_yaw is not None and self.pose_cov_yaw < 0.1:
                status_parts.append("⏳COV")
            else:
                status_parts.append("❌COV")

            status_parts.append(yaw_status)
            status_parts.append("✅VEL" if twist_ok else "⏳VEL")
            status = " ".join(status_parts)

            self.get_logger().info(
                f"{now - self.start_time:7.1f}s  "
                f"{math.degrees(yaw):8.1f}  "
                f"{yaw_rate:+8.4f}  "
                f"{self.pose_cov_yaw:12.6f}  "
                f"{self.twist_cov_vx:12.6f}  "
                f"{status}"
            )


def main():
    parser = argparse.ArgumentParser(description="EKF Filter Monitor")
    parser.add_argument("--simple", action="store_true", help="简洁输出模式")
    args = parser.parse_args()

    rclpy.init()
    node = EKFMonitor(simple=args.simple)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n=== 监控结束 ===")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
