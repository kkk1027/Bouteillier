"""
geometry.py
点云提取与聚类 —— 纯函数，不依赖 rclpy，可以脱离 ROS 单测。
"""

import math


def wrap_angle(a):
    """把角度归一化到 [-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def rear_offset(a):
    """方位角到「正后方」的角距离，返回值越小越接近正后方。

    0°  → pi（正前方，离后方最远）
    ±180° → 0（正后方）
    """
    return abs(abs(wrap_angle(a)) - math.pi)


def extract_points(msg, max_range):
    """从 LaserScan 提取**全向**有效点，返回 [(距离, 角度), ...]，按角度升序。

    注意这里是全向的，不做扇区过滤。
    原因：后退前要检查后方，如果在这里就限成 ±30°，
    后方检查永远拿不到数据，会误判「后方无障碍」而盲目后退。

    过滤条件：
      * NaN / Inf          —— 雷达无效值
      * 小于 range_min      —— 低于量程下限（本机 0.3m）
      * 大于 max_range      —— 太远，不是跟随目标，也没必要参与安全判断
    """
    pts = []
    amin = msg.angle_min
    inc = msg.angle_increment
    rmin = msg.range_min if msg.range_min > 0.0 else 0.05
    rmax = min(max_range, msg.range_max)

    for i, r in enumerate(msg.ranges):
        if math.isnan(r) or math.isinf(r):
            continue
        if r < rmin or r > rmax:
            continue
        pts.append((float(r), wrap_angle(amin + i * inc)))

    pts.sort(key=lambda t: t[1])
    return pts


def sector_filter(pts, half_angle_rad):
    """只保留 |角度| <= half_angle_rad 的点（前方扇区）。"""
    return [p for p in pts if abs(p[1]) <= half_angle_rad]


def cluster_points(pts, angle_gap_rad, range_gap):
    """把相邻点合并成物体簇。

    判定：相邻两点的角度差 <= angle_gap 且 距离差 <= range_gap 时算同一个物体。

    返回 [{'dist': 簇内最近距离, 'angle': 该最近点的角度,
           'n': 点数, 'angle_center': 簇中心角度}, ...]

    注意 'dist' 取簇内**最近点**而不是平均值：
    控制目标是「别撞上去」，关心的是最近的那一面。
    """
    if not pts:
        return []

    groups = []
    cur = [pts[0]]
    for prev, item in zip(pts, pts[1:]):
        if (abs(item[1] - prev[1]) <= angle_gap_rad
                and abs(item[0] - prev[0]) <= range_gap):
            cur.append(item)
        else:
            groups.append(cur)
            cur = [item]
    groups.append(cur)

    out = []
    for g in groups:
        if not g:
            continue
        nearest = min(g, key=lambda t: t[0])
        out.append({
            'dist': nearest[0],
            'angle': nearest[1],
            'n': len(g),
            'angle_center': sum(t[1] for t in g) / len(g),
        })
    return out


def select_target(clusters, locked, ref_dist, ref_angle,
                  lock_max_range, track_max_range,
                  min_points=1, max_jump=None):
    """选目标簇。

    未锁定：取最近簇，且必须在 lock_max_range 内。
    已锁定：在 track_max_range 内，取「最像上次目标」的簇。

    参数：
        min_points  忽略点数少于这个值的簇。
                    ★ 实测必须用：真实雷达里 1~3 个点的簇几乎都是
                    噪声（柜子边缘、桌腿反光），但它们的距离可能只有
                    0.8m，一旦被选中就会让车朝它冲过去。
        max_jump    已锁定时，若新目标的距离与当前目标相差超过此值，
                    判为「换了个物体」→ 返回 None（当作本帧没目标），
                    交给丢帧逻辑处理，而不是立刻切过去。
                    ★ 实测必须用：目标被推到 30cm 内（雷达量程下限）
                    时近处点全部消失，剩下的远簇距离可能只差 0.54m，
                    不拦住就会直接切换目标并开始前进。

    返回 (dist, angle, n) 或 None。
    """
    if not clusters:
        return None

    # 过滤过小的簇
    if min_points > 1:
        clusters = [c for c in clusters if c['n'] >= min_points]
        if not clusters:
            return None

    if not locked:
        cand = [c for c in clusters if c['dist'] <= lock_max_range]
        if not cand:
            return None
        c = min(cand, key=lambda x: x['dist'])
        return c['dist'], c['angle'], c['n']

    cand = [c for c in clusters if c['dist'] <= track_max_range]
    if not cand:
        return None

    if ref_dist > 0.0:
        def score(c):
            dd = abs(c['dist'] - ref_dist)
            da = abs(wrap_angle(c['angle'] - ref_angle))
            # 角度权重给高：距离相近的两个点，靠角度区分
            return dd + 2.0 * da
        best = min(cand, key=score)
    else:
        best = min(cand, key=lambda x: x['dist'])

    # 距离跳变检测：与原目标差太多说明不是同一个物体
    if ref_dist > 0.0 and max_jump is not None:
        if abs(best['dist'] - ref_dist) > max_jump:
            return None

    return best['dist'], best['angle'], best['n']


def rear_is_clear(all_clusters, rear_half_rad, rear_range):
    """后方扇区内是否有障碍。

    没有点云数据时返回 False（按「不安全」处理）——
    宁可不后退，也不要盲退。

    后方扇区 = 方位角接近 ±180° 的区域，例如 rear_half_rad=40°
    对应 [140°, 180°] 与 [-180°, -140°]。
    """
    if not all_clusters:
        return False
    for c in all_clusters:
        if rear_offset(c['angle']) <= rear_half_rad and c['dist'] < rear_range:
            return False
    return True


def front_nearest(front_clusters):
    """前方最近障碍距离，没有则返回 None。"""
    if not front_clusters:
        return None
    best = None
    for c in front_clusters:
        if best is None or c['dist'] < best:
            best = c['dist']
    return best
