#!/usr/bin/env python3
"""只读诊断：把当前激光与地图做全局暴力扫描匹配，判断 AMCL 是否卡错。

不发布任何运动指令，不修改任何状态。

输出：
  1) AMCL 当前位姿的匹配得分
  2) 全图暴力搜索找到的最优位姿及其得分
  3) 两者的距离差 —— 差距大说明 AMCL 卡在假最小值
  4) 得分地形：AMCL 位姿附近是否为平台区（平坦 = 无法区分）
"""
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def chamfer_dt(occ: np.ndarray) -> np.ndarray:
    """两遍扫描的 chamfer 距离变换（单位：像素）。"""
    INF = 1e9
    h, w = occ.shape
    d = np.where(occ, 0.0, INF)
    # 前向
    for y in range(h):
        row = d[y]
        prev = d[y - 1] if y > 0 else None
        for x in range(w):
            v = row[x]
            if v == 0:
                continue
            best = v
            if x > 0:
                best = min(best, row[x - 1] + 1.0)
                if prev is not None:
                    best = min(best, prev[x - 1] + 1.41421)
            if prev is not None:
                best = min(best, prev[x] + 1.0)
                if x + 1 < w:
                    best = min(best, prev[x + 1] + 1.41421)
            row[x] = best
    # 反向
    for y in range(h - 1, -1, -1):
        row = d[y]
        nxt = d[y + 1] if y + 1 < h else None
        for x in range(w - 1, -1, -1):
            v = row[x]
            best = v
            if x + 1 < w:
                best = min(best, row[x + 1] + 1.0)
                if nxt is not None:
                    best = min(best, nxt[x + 1] + 1.41421)
            if nxt is not None:
                best = min(best, nxt[x] + 1.0)
                if x > 0:
                    best = min(best, nxt[x - 1] + 1.41421)
            row[x] = best
    return d


def main():
    rclpy.init()
    node = Node('scan_match_diag')

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)

    store = {}

    def map_cb(m):
        store['map'] = m
        store['map_t'] = node.get_clock().now()

    def scan_cb(m):
        store['scan'] = m
        store['scan_t'] = node.get_clock().now()

    node.create_subscription(OccupancyGrid, '/map', map_cb, latched)
    node.create_subscription(LaserScan, '/scan', scan_cb, sensor)

    tf_buf = Buffer()
    TransformListener(tf_buf, node)

    print('等待地图/激光...', flush=True)
    for _ in range(400):
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(k in store for k in ('map', 'scan')):
            break
    missing = [k for k in ('map', 'scan') if k not in store]
    if missing:
        print('❌ 缺少数据: %s' % missing, flush=True)
        return 1
    print('数据齐备\n', flush=True)

    grid = store['map']
    w = grid.info.width
    h = grid.info.height
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    raw = np.array(grid.data, dtype=np.int8).reshape(h, w)   # 行 0 = 世界 y 最大
    free_mask = raw == 0
    occ_mask = raw >= 65
    unk_mask = raw < 0
    print('地图 %dx%d @ %.3fm  空闲 %.1f%%  墙 %.1f%%  未知 %.1f%%'
          % (w, h, res, free_mask.mean() * 100, occ_mask.mean() * 100,
             unk_mask.mean() * 100), flush=True)

    # 未知区不参与匹配；空闲区记为不可通行惩罚
    dt_occ = chamfer_dt(occ_mask) * res        # 到最近墙的距离(m)
    # 未知区给出中等惩罚，避免粒子飘到未知区得高分
    dt_unk = chamfer_dt(~unk_mask) * res

    scan = store['scan']
    stamp = scan.header.frame_id
    # TF 需要几秒钟才能从 robot_state_publisher 汇齐（URDF + joint_states），
    # 这里必须带重试地 spin，否则首次 lookup 会直接抛
    # "source_frame does not exist"。
    tf = None
    for _ in range(60):
        try:
            tf = tf_buf.lookup_transform(
                'base_link', stamp, rclpy.time.Time(seconds=0, nanoseconds=0),
                timeout=rclpy.duration.Duration(seconds=0.5))
            break
        except Exception:                                # noqa: BLE001
            rclpy.spin_once(node, timeout_sec=0.1)
    if tf is None:
        print('❌ 等待 %d 次仍无法获取 base_link -> %s 变换。'
              '检查 robot_state_publisher 与 urdf2tf 是否在跑。'
              % (60, stamp), flush=True)
        return 1
    tx = tf.transform.translation.x
    ty = tf.transform.translation.y
    q = tf.transform.rotation
    tyaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))
    print('激光系 %s 相对 base_link: (%.3f, %.3f, %.1f°)'
          % (stamp, tx, ty, math.degrees(tyaw)), flush=True)

    ranges = np.array(scan.ranges, dtype=np.float64)
    ang = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    rmax = min(scan.range_max, 8.0)
    good = np.isfinite(ranges) & (ranges >= max(scan.range_min, 0.15)) & (ranges <= rmax)
    # 均匀抽样，保留约 120 束
    idx = np.where(good)[0]
    step = max(1, len(idx) // 120)
    idx = idx[::step]
    print('激光 %d 束，其中有效 %d，抽样 %d 束用于匹配'
          % (len(ranges), int(good.sum()), len(idx)), flush=True)

    lx = ranges[idx] * np.cos(ang[idx]) + tx
    ly = ranges[idx] * np.sin(ang[idx]) + ty
    pts = np.stack([lx, ly], axis=1)                  # base_link 系

    def score(poses):
        """poses: (N,3) [x, y, yaw]。返回 (N,) 平均代价，越小越好。"""
        N = poses.shape[0]
        out = np.full(N, np.inf)
        c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
        for k in range(N):
            px = c[k] * pts[:, 0] - s[k] * pts[:, 1] + poses[k, 0]
            py = s[k] * pts[:, 0] + c[k] * pts[:, 1] + poses[k, 1]
            cx = ((px - ox) / res).astype(np.int32)
            cy = ((py - oy) / res).astype(np.int32)
            cy_img = h - 1 - cy                        # 行翻转
            inb = (cx >= 0) & (cx < w) & (cy_img >= 0) & (cy_img < h)
            if inb.sum() == 0:
                continue
            d = dt_occ[cy_img[inb], cx[inb]]
            du = dt_unk[cy_img[inb], cx[inb]]
            pen = 2.0 * (h * res)                      # 出界重罚
            total = d.sum() + 0.5 * du.sum() + pen * (~inb).sum()
            out[k] = total / len(pts)
        return out

    # 当前估计位姿：从 TF 取 map->base_link。
    # 不用 /amcl_pose：收敛后 auto_localizer 会停止请求 nomotion 更新，
    # AMCL 就不再发布 /amcl_pose，但 TF 里的 map->odom 仍然有效。
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            m2b = tf_buf.lookup_transform('map', 'base_link',
                                           rclpy.time.Time())
            break
        except Exception:                              # noqa: BLE001
            m2b = None
    if m2b is None:
        print('❌ 无法获取 map -> base_link（AMCL 可能还没有定位结果）',
              flush=True)
        return 1
    mp = m2b.transform.translation
    ap_yaw = math.atan2(2 * (m2b.transform.rotation.w * m2b.transform.rotation.z
                             + m2b.transform.rotation.x * m2b.transform.rotation.y),
                        1 - 2 * (m2b.transform.rotation.y ** 2
                                 + m2b.transform.rotation.z ** 2))
    print('当前估计位姿（TF map->base_link）=(%.3f, %.3f, %.1f°)'
          % (mp.x, mp.y, math.degrees(ap_yaw)), flush=True)
    amcl_score = score(np.array([[mp.x, mp.y, ap_yaw]]))[0]
    print('该位姿的平均墙距代价 = %.4f m' % amcl_score, flush=True)

    # ---------- 全局暴力搜索 ----------
    xs = np.arange(ox + res, ox + w * res, 0.10)
    ys = np.arange(oy + res, oy + h * res, 0.10)
    yaws = np.deg2rad(np.arange(-180, 180, 5.0))
    print('全局搜索 %d 位置 x %d 角度 = %d 次评估...'
          % (len(xs), len(yaws), len(xs) * len(ys) * len(yaws)), flush=True)

    best = (np.inf, None)
    for yaw in yaws:
        c, s = math.cos(yaw), math.sin(yaw)
        px = c * pts[:, 0] - s * pts[:, 1]
        py = s * pts[:, 0] + c * pts[:, 1]
        cx0 = (px / res)
        cy0 = (py / res)
        gx, gy = np.meshgrid(xs, ys)
        cx = ((gx - ox) / res + cx0).astype(np.int32)      # (ny, nx, npts)
        cy = ((gy - oy) / res + cy0).astype(np.int32)
        cyi = h - 1 - cy
        inb = (cx >= 0) & (cx < w) & (cyi >= 0) & (cyi < h)
        safe_x = np.clip(cx, 0, w - 1)
        safe_y = np.clip(cyi, 0, h - 1)
        d = np.where(inb, dt_occ[safe_y, safe_x], 3.0)
        du = np.where(inb, dt_unk[safe_y, safe_x], 3.0)
        s_tot = d.sum(axis=2) + 0.5 * du.sum(axis=2)
        k = np.unravel_index(np.argmin(s_tot), s_tot.shape)
        v = s_tot[k] / len(pts)
        if v < best[0]:
            best = (v, (xs[k[1]], ys[k[0]], yaw))

    print('全局最优 (%.2f, %.2f, %.1f°)  代价 = %.4f m'
          % (best[1][0], best[1][1], math.degrees(best[1][2]), best[0]),
          flush=True)

    dxy = math.hypot(best[1][0] - mp.x, best[1][1] - mp.y)
    dyaw = abs(math.degrees(best[1][2]) - math.degrees(ap_yaw))
    dyaw = min(dyaw, 360 - dyaw)
    print('\n差距：位置 %.2f m，朝向 %.1f°' % (dxy, dyaw), flush=True)
    if dxy < 0.4 and dyaw < 20:
        print('✅ 当前位姿与全局最优一致 —— 定位没有失效')
    else:
        print('❌ 当前位姿不是最优匹配 —— AMCL 已跟丢，需要重新全局定位')

    # ---------- 区分度：位姿附近得分地形 ----------
    probe = []
    for ddx in np.arange(-0.5, 0.51, 0.25):
        for ddy in np.arange(-0.5, 0.51, 0.25):
            probe.append([mp.x + ddx, mp.y + ddy, ap_yaw])
    probe = np.array(probe)
    sc = score(probe)
    print('\nAMCL 位姿附近 ±0.5m 的得分（标准差的平方 = 似然场区分度）:',
          flush=True)
    print('  均值 %.4f  标准差 %.4f  min %.4f  max %.4f'
          % (sc.mean(), sc.std(), sc.min(), sc.max()), flush=True)
    if sc.std() < 0.02:
        print('  ⚠️ 得分几乎不变 —— 该处几何过于对称/空洞，AMCL 无法区分位置')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
