#!/usr/bin/env python3
"""test_loc_quality_node.py
============================================================
auto_localizer 的**节点级**端到端测试：验证 /localization_status 真的
能发出来、内容真的正确、并且能在「σ 超线但匹配率高」时判收敛。

与 test_loc_quality.py 的分工：
    test_loc_quality.py       纯算法（不需要 ROS，毫秒级）
    test_loc_quality_node.py  真节点接线（需要 rclpy，约 20 秒）

★ 安全边界 ★
    - 强制 ROS_DOMAIN_ID=99 + ROS_LOCALHOST_ONLY=1，与实车完全隔离，
      看不见也影响不到真实机器人。
    - 本脚本**不发布任何 /cmd_vel**，不启动导航，不碰底盘。
    - 所有输入（地图 / 激光 / 位姿）都是本脚本自己造出来的。
"""

import json
import math
import os
import sys
import threading
import time

# ---- 必须在 import rclpy 之前设置，否则无效 ----
os.environ['ROS_DOMAIN_ID'] = '99'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = os.environ.get(
    'RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')

import rclpy                                        # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped   # noqa: E402
from nav_msgs.msg import OccupancyGrid              # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node                         # noqa: E402
from rclpy.qos import (DurabilityPolicy, HistoryPolicy,  # noqa: E402
                       QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import LaserScan               # noqa: E402
from std_msgs.msg import String                     # noqa: E402
from std_srvs.srv import Empty                      # noqa: E402

ok = 0
total = 0


def check(name, cond, extra=''):
    global ok, total
    total += 1
    ok += bool(cond)
    print('  %s %s %s' % ('✅' if cond else '❌', name, extra))


# ============================================================
# 造一张 4m x 4m 地图：四周墙 + 中间竖墙
#   （和 test_loc_quality.py 同一张图，便于交叉验证）
# ============================================================
RES, W, H = 0.05, 80, 80
NBEAM, AMIN, AINC, RMAX = 471, -math.pi, 2 * math.pi / 471, 12.0
TRUE_POSE = (1.0, 1.0, 0.0)


def make_map():
    data = [0] * (W * H)
    for i in range(W):
        data[i] = 100
        data[(H - 1) * W + i] = 100
    for j in range(H):
        data[j * W] = 100
        data[j * W + W - 1] = 100
    for j in range(20, 60):
        data[j * W + 40] = 100
    return data


MAP_DATA = make_map()


def simulate_ranges(x, y, yaw):
    out = []
    for i in range(NBEAM):
        a = yaw + AMIN + i * AINC
        ca, sa = math.cos(a), math.sin(a)
        d, hit = RES * 0.5, RMAX
        while d < RMAX:
            col = int((x + d * ca) / RES)
            row = int((y + d * sa) / RES)
            if not (0 <= col < W and 0 <= row < H):
                break
            if MAP_DATA[row * W + col] >= 65:
                hit = d
                break
            d += RES * 0.5
        out.append(hit)
    return out


# ============================================================
# 假环境：地图 / 激光 / AMCL 位姿 / 两个服务
# ============================================================
class FakeWorld(Node):
    """发布"完美的"传感器数据，让 auto_localizer 有机会收敛。"""

    def __init__(self):
        super().__init__('fake_world')
        latched = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.create_service(Empty, '/reinitialize_global_localization',
                            self._on_reinit)
        self.create_service(Empty, '/request_nomotion_update',
                            self._on_nomotion)
        self.reinit_calls = 0
        self.nomotion_calls = 0

        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = 'map'
        self.map_msg.info.resolution = RES
        self.map_msg.info.width = W
        self.map_msg.info.height = H
        self.map_msg.info.origin.position.x = 0.0
        self.map_msg.info.origin.position.y = 0.0
        self.map_msg.data = MAP_DATA

        self.ranges = simulate_ranges(*TRUE_POSE)
        # 事故里的 σ 水平：σ_xy=0.112m σ_yaw=5.2° —— 恰好超收敛线 (0.08/0.05)
        self.std_xy = 0.112
        self.std_yaw = math.radians(5.2)

    def _on_reinit(self, request, response):
        self.reinit_calls += 1
        return response

    def _on_nomotion(self, request, response):
        self.nomotion_calls += 1
        return response

    def publish_all(self):
        self.map_pub.publish(self.map_msg)

        s = LaserScan()
        s.header.frame_id = 'laser_link'
        s.header.stamp = self.get_clock().now().to_msg()
        s.angle_min = AMIN
        s.angle_increment = AINC
        s.angle_max = AMIN + (NBEAM - 1) * AINC
        s.range_min = 0.05
        s.range_max = RMAX
        s.ranges = self.ranges
        self.scan_pub.publish(s)

        p = PoseWithCovarianceStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.pose.position.x = TRUE_POSE[0]
        p.pose.pose.position.y = TRUE_POSE[1]
        p.pose.pose.orientation.z = math.sin(TRUE_POSE[2] / 2.0)
        p.pose.pose.orientation.w = math.cos(TRUE_POSE[2] / 2.0)
        p.pose.covariance[0] = self.std_xy ** 2
        p.pose.covariance[7] = self.std_xy ** 2
        p.pose.covariance[35] = self.std_yaw ** 2
        self.pose_pub.publish(p)


class StatusCollector(Node):
    """收集 /localization_status。"""

    def __init__(self):
        super().__init__('status_collector')
        self.msgs = []
        self.create_subscription(String, '/localization_status',
                                 self._cb, 10)

    def _cb(self, msg):
        try:
            self.msgs.append(json.loads(msg.data))
        except Exception:                # noqa: BLE001
            self.msgs.append({'__bad_json__': msg.data[:120]})

    def last(self):
        return self.msgs[-1] if self.msgs else None


# ============================================================
def main():
    global ok, total
    print('=== 启动隔离的 ROS 图（domain 99，本机回环） ===')
    rclpy.init()

    world = FakeWorld()
    coll = StatusCollector()
    exe = SingleThreadedExecutor()
    exe.add_node(world)
    exe.add_node(coll)

    # 导入真实节点
    try:
        from fishbot_navigation2.auto_localizer import AutoLocalizer
    except Exception as exc:                # noqa: BLE001
        print('❌ 无法导入 AutoLocalizer: %s' % exc)
        print('   （请先 colcon build --packages-select fishbot_navigation2）')
        return 2

    loc = None
    try:
        loc = AutoLocalizer()
    except Exception as exc:                # noqa: BLE001
        print('❌ AutoLocalizer 构造失败: %s' % exc)
        import traceback
        traceback.print_exc()
        return 2
    exe.add_node(loc)

    print('✅ AutoLocalizer 构造成功（参数声明全部通过）')
    total += 1
    ok += 1

    # 参数检查
    params = loc._assessor
    check('匹配率判定器已就绪', params is not None)
    check('匹配器已就绪', loc._matcher is not None)
    check('稳定性跟踪器已就绪', loc._stability is not None)

    stop = threading.Event()

    def spin():
        while not stop.is_set():
            world.publish_all()
            try:
                exe.spin_once(timeout_sec=0.1)
            except Exception:              # noqa: BLE001
                pass

    th = threading.Thread(target=spin, daemon=True)
    th.start()

    print()
    print('=== 等待 auto_localizer 收敛（最多 60 秒） ===')
    deadline = time.time() + 60.0
    converged_at = None
    first_status_at = None
    while time.time() < deadline:
        if coll.msgs and first_status_at is None:
            first_status_at = time.time()
        s = coll.last()
        if s and s.get('ready'):
            converged_at = time.time()
            break
        time.sleep(0.3)

    stop.set()
    time.sleep(0.3)

    print()
    print('=== 结果 ===')
    check('收到了 /localization_status', bool(coll.msgs),
          '共 %d 条' % len(coll.msgs))

    bad = [m for m in coll.msgs if '__bad_json__' in m]
    check('状态是合法 JSON', not bad,
          ('坏帧 %d 条' % len(bad)) if bad else '')

    s = coll.last() or {}
    check('状态含 state 字段', 'state' in s, str(s.get('state')))
    check('状态含 ready 字段', 'ready' in s)
    check('状态含 progress', 'progress' in s, str(s.get('progress')))
    check('状态含 match_rate', 'match_rate' in s, str(s.get('match_rate')))
    check('状态含 std_xy', 'std_xy' in s, str(s.get('std_xy')))
    check('状态含 stable', 'stable' in s)
    check('状态含 pose', 'pose' in s and isinstance(s.get('pose'), dict))

    print('    最后一条状态:')
    for k in ('state', 'ready', 'progress', 'std_xy', 'match_rate',
              'stable', 'stable_sec', 'reason'):
        print('      %-11s = %s' % (k, s.get(k)))

    check('★ 匹配率被算出来了（不是 None）',
          s.get('match_rate') is not None, str(s.get('match_rate')))
    if s.get('match_rate') is not None:
        check('★ 正确位姿下匹配率很高（>0.9）',
              s['match_rate'] > 0.90, '%.3f' % s['match_rate'])
    check('★ σ 确实超了收敛线（复刻事故条件）',
          s.get('std_xy') is not None and s['std_xy'] > loc.converge_cov_xy,
          'σ=%.3f > %.3f' % (s.get('std_xy') or 0, loc.converge_cov_xy))
    check('★ σ 超线但仍然判定收敛（C 的核心）',
          converged_at is not None,
          '耗时 %.1fs' % ((converged_at - first_status_at)
                          if converged_at and first_status_at else -1))
    check('★ 全程没有触发全局重撒（不再 120s 循环）',
          world.reinit_calls <= 1,
          'reinitialize 被调用 %d 次' % world.reinit_calls)
    check('节点没有崩（状态还在发）', len(coll.msgs) >= 5,
          '%d 条' % len(coll.msgs))

    # 清理
    try:
        loc._timer.cancel()
        exe.remove_node(loc)
        exe.shutdown()
        loc.destroy_node()
        world.destroy_node()
        coll.destroy_node()
    except Exception:                     # noqa: BLE001
        pass
    try:
        rclpy.shutdown()
    except Exception:                     # noqa: BLE001
        pass

    print()
    print('=' * 60)
    print('通过 %d / %d' % (ok, total))
    print('=' * 60)
    return 0 if ok == total else 1


if __name__ == '__main__':
    sys.exit(main())
