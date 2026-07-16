#!/usr/bin/env python3
"""
QoS 桥接节点：订阅 /scan (BEST_EFFORT) → 重发布 /scan (RELIABLE)

ydlidar_node 以 SensorData QoS (BEST_EFFORT) 发布激光数据，
但 Cartographer 和 Nav2 需要 RELIABLE QoS。
此节点订阅 /scan (BEST_EFFORT) 并以 RELIABLE QoS 转发到 /scan。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanQoSBridge(Node):
    def __init__(self):
        super().__init__('scan_qos_bridge')

        # BEST_EFFORT 订阅（匹配 ydlidar 发布的 SensorData QoS）
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # RELIABLE 发布（匹配 Cartographer / Nav2 订阅端）
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.callback, sub_qos)
        self.pub = self.create_publisher(
            LaserScan, '/scan', pub_qos)

        self.get_logger().info('✅ QoS 桥接: /scan (BEST_EFFORT) → /scan (RELIABLE)')

    def callback(self, msg):
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanQoSBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
