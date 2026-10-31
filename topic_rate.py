#!/usr/bin/env python3
"""QoS 自适应的 ROS 2 话题测速/探活工具。

为什么需要它
------------
`ros2 topic hz` / `ros2 topic echo` 在 Humble 里默认用 **RELIABLE** 订阅，
而本项目 ESP32(micro-ROS) 发的 `/odom`、`/imu_raw` 是 **BEST_EFFORT**。

QoS 兼容规则：
  Publisher RELIABLE   + Subscriber BEST_EFFORT  → 兼容
  Publisher BEST_EFFORT + Subscriber RELIABLE   → **不兼容，收不到任何数据**

于是 `ros2 topic hz /odom` 明明有数据也报 "no new messages"，
把人引向"ESP32 没连上"的错误方向（本项目实际踩过这个坑）。

而 `ros2 topic hz` 在 Humble 又没有 --qos-reliability 参数，
所以只能用 rclpy 自己写订阅者。本脚本会依次尝试几种常见 QoS 组合，
哪一种能收到数据就用哪一种。

用法
----
  python3 topic_rate.py /odom            # 默认测 3 秒
  python3 topic_rate.py /odom 5          # 测 5 秒
  python3 topic_rate.py /map 3           # 也会自动尝试 TRANSIENT_LOCAL

输出（便于 shell 判断）
----
  OK hz=9.98 count=30 qos=best_effort/volatile
  EMPTY count=0                    # 3 秒内一帧都没收到
  NOTOPIC                          # 话题在图上不存在
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# 组合顺序有讲究：先试本项目最常用的组合，命中即返回
COMBOS = [
    ('best_effort', 'volatile', ReliabilityPolicy.BEST_EFFORT,
     DurabilityPolicy.VOLATILE),
    ('reliable', 'transient_local', ReliabilityPolicy.RELIABLE,
     DurabilityPolicy.TRANSIENT_LOCAL),
    ('reliable', 'volatile', ReliabilityPolicy.RELIABLE,
     DurabilityPolicy.VOLATILE),
    ('best_effort', 'transient_local', ReliabilityPolicy.BEST_EFFORT,
     DurabilityPolicy.TRANSIENT_LOCAL),
]


def try_once(topic: str, seconds: float, rel, dur,
             rel_name: str, dur_name: str):
    """用指定 QoS 订阅 seconds 秒，返回 (count, hz)。"""
    rclpy.init()
    node = Node('topic_rate_probe')
    counter = {'n': 0, 'first': None, 'last': None}

    def cb(_msg):
        now = time.time()
        counter['n'] += 1
        if counter['first'] is None:
            counter['first'] = now
        counter['last'] = now

    qos = QoSProfile(depth=10, reliability=rel, durability=dur)
    # 通用类型：任何消息都能收（rclpy 需要具体类型，这里用通配实现不了，
    # 所以改用 ros2 命令行等价方式不可行 —— 改为探测 topic type 后动态导入）
    node.create_subscription(_resolve_type(topic), topic, cb, qos)

    end = time.time() + seconds
    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    n = counter['n']
    hz = 0.0
    if n >= 2 and counter['last'] and counter['first']:
        span = counter['last'] - counter['first']
        if span > 0:
            hz = (n - 1) / span
    return n, hz


def _resolve_type(topic: str):
    """查询话题的消息类型并导入对应的 Python 类。

    `ros2 topic type` 返回三段式字符串，例如 ``nav_msgs/msg/Odometry``。
    必须按 / 切成三段：包名 / 接口类别(msg|srv|action) / 类型名。
    """
    import subprocess
    out = subprocess.run(
        ['ros2', 'topic', 'type', topic],
        capture_output=True, text=True, timeout=20).stdout.strip()
    parts = out.split('/')
    if len(parts) != 3:
        raise RuntimeError('NOTOPIC: %r' % out)
    pkg, kind, name = parts
    mod = __import__('%s.%s' % (pkg, kind), fromlist=[name])
    return getattr(mod, name)


def main():
    if len(sys.argv) < 2:
        print('用法: topic_rate.py <topic> [seconds]')
        return 2
    topic = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

    try:
        _resolve_type(topic)
    except Exception:
        print('NOTOPIC')
        return 1

    for rel_name, dur_name, rel, dur in COMBOS:
        try:
            n, hz = try_once(topic, seconds, rel, dur, rel_name, dur_name)
        except Exception as exc:  # noqa: BLE001
            print('ERROR %s' % exc)
            return 1
        if n > 0:
            print('OK hz=%.2f count=%d qos=%s/%s'
                  % (hz, n, rel_name, dur_name))
            return 0

    print('EMPTY count=0')
    return 1


if __name__ == '__main__':
    sys.exit(main())
