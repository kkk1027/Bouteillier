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
        angs = [t[1] for t in g]
        out.append({
            'dist': nearest[0],
            'angle': nearest[1],
            'n': len(g),
            'angle_center': sum(angs) / len(angs),
            # 簇的角度宽度。人腿很窄（几度），墙/柜子很宽（几十度），
            # 配对时用它把「墙」排除掉。
            'span': max(angs) - min(angs),
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


# ============================================================
# 双腿成对锁定
# ------------------------------------------------------------
# 为什么需要：
#     雷达装在车体下沿（离地约 10cm），扫到人时**扫不到躯干**，
#     只有两条小腿。两条腿在极坐标里是两个互不相连的小簇，
#     每条腿通常只有 1~4 个点。
#
#     旧的单簇逻辑要求「一个簇至少 5 个点」才认，这个人腿的工况
#     直接整簇被当噪声丢掉 —— 于是永远锁不上。
#
#     就算偶尔某条腿凑够 5 个点被锁定，目标也会在两条腿之间
#     来回跳，角度瞬间抖 ±10°，车跟着抽。
#
# 解决办法：
#     先找「成对的两个小簇」，把两点**中点**当目标。
#     中点约等于人的身体中心线，既不会左右跳，也正是
#     「保持 50cm 距离」该量到的地方。
# ============================================================

def to_xy(dist, angle):
    """极坐标 → 直角坐标。x 向前，y 向左（雷达坐标系）。"""
    return dist * math.cos(angle), dist * math.sin(angle)


def _pair_of(a, b):
    """由两个簇构造「双腿对」。

    返回 dict，与单簇目标的字段保持兼容（dist/angle/n/kind），
    额外带上 sep（两点直线距离）和两条腿各自的读数，方便打日志。

    还额外分解出两点连线在中点视线方向上的投影：
        lateral  横向间距（垂直于视线）—— 就是「腿距」，
                 真人约 0.22m，极少超过 0.5m
        along    纵向间距（沿视线）  —— 人侧身站时会有值
    这两个分量能把「随机噪声凑成的两点」排除掉：
    噪声两点往往张得很开、而且每帧都在变。
    """
    xa, ya = to_xy(a['dist'], a['angle'])
    xb, yb = to_xy(b['dist'], b['angle'])

    xm = 0.5 * (xa + xb)
    ym = 0.5 * (ya + yb)
    dm = math.hypot(xm, ym)
    if dm <= 1e-6:
        return None

    ang = math.atan2(ym, xm)
    ux, uy = math.cos(ang), math.sin(ang)      # 中点视线方向
    dx, dy = xb - xa, yb - ya

    return {
        'dist': dm,
        'angle': ang,
        'n': int(a['n']) + int(b['n']),
        'kind': 'pair',
        'sep': math.hypot(dx, dy),
        'lateral': abs(-dx * uy + dy * ux),
        'along': abs(dx * ux + dy * uy),
        'n_a': int(a['n']),
        'n_b': int(b['n']),
        'leg_a': (a['dist'], a['angle']),
        'leg_b': (b['dist'], b['angle']),
    }


def find_leg_pairs(clusters, max_range, max_sep, min_sep,
                   max_dist_diff, min_points_total,
                   max_span_deg=45.0, max_clusters=24,
                   max_lateral_sep=0.50, max_along_sep=0.25):
    """在候选簇里找所有合格的「双腿对」，按中点距离升序返回。

    判定条件（全部满足才算一对）：
        1. 两个簇各自的最近点都在 max_range 内
        2. 两点直线距离在 [min_sep, max_sep] 内（默认 5cm ~ 60cm）
        3. 两点到雷达的距离差 <= max_dist_diff
           —— 同一人的两条腿基本处在同一深度；
           一近一远的两点是「腿 + 墙」，不是一双腿
        4. 两个簇点数之和 >= min_points_total
           —— 单独一个点太容易是噪声，至少凑够 2 个点
        5. 两个簇的角度宽度都 <= max_span_deg
           —— 人腿很窄（几度），墙/柜子很宽，靠这个把墙排除掉
        6. 两点的**横向**间距 <= max_lateral_sep
           —— 横向就是「腿距」，真人约 0.22m，极少超过 0.5m
        7. 两点的**纵向**错位 <= max_along_sep
           —— 并排站时两腿几乎前后对齐；侧身站时也会前后重叠。
           真正一前一后差 30cm 以上的，只能是两个不相关的物体。

    第 6、7 条专治「帧间随机跳动的噪声点凑成一对」：
    噪声两点既可能横向张得很开，也可能一前一后错得很开。

    参数：
        max_range        两个点各自的距离上限（未锁定时会传 1.0m，
                         因为人可能在 1m 外才被认出来）
        max_clusters     最多看最近的多少个簇（O(n^2) 性能保护）
    """
    if len(clusters) < 2:
        return []

    cand = []
    span_limit = math.radians(max_span_deg)
    for c in clusters:
        if c['dist'] > max_range:
            continue
        span = c.get('span')
        if span is not None and span > span_limit:
            continue
        cand.append(c)

    if len(cand) < 2:
        return []

    cand.sort(key=lambda c: c['dist'])
    cand = cand[:max_clusters]

    pairs = []
    for i in range(len(cand)):
        a = cand[i]
        for j in range(i + 1, len(cand)):
            b = cand[j]

            if a['n'] + b['n'] < min_points_total:
                continue
            if abs(a['dist'] - b['dist']) > max_dist_diff:
                continue

            p = _pair_of(a, b)
            if p is None:
                continue
            if p['sep'] < min_sep or p['sep'] > max_sep:
                continue
            if max_lateral_sep is not None and p['lateral'] > max_lateral_sep:
                continue
            if max_along_sep is not None and p['along'] > max_along_sep:
                continue
            pairs.append(p)

    pairs.sort(key=lambda p: p['dist'])
    return pairs


def find_single_leg(clusters, ref_dist, ref_angle, max_range,
                    max_points=4, max_dist_diff=0.35,
                    max_angle_diff=None, max_span_deg=45.0):
    """已锁定时「只看到一条腿」的兜底候选，没有则返回 None。

    为什么需要：
        人走路时两条腿会互相遮挡，经常只有一条腿可见。
        这时双腿对不成立，而普通的单簇路径又因为
        min_cluster_points=5 把这些 1~4 点的小簇过滤掉了，
        结果就是目标连续丢帧、车一顿一顿地停。

    为什么不能直接用单腿的 (距离, 角度)：
        单腿偏离人体中心约半个腿距（22cm/2 = 11cm），
        在 0.6m 处就相当于 ±10°。直接拿来控制会让车左右抽。
        所以返回 angle_valid=False —— 调用方只用它的**距离**，
        角度交给卡尔曼预测值。距离仍然是可靠的：
        两条腿的深度基本一致。

    参数：
        max_points      只认小簇。5 点以上的大簇属于实体目标，
                        走正常单簇路径，不该进兜底
        max_dist_diff   与预测距离的容差
        max_angle_diff  与预测角度的容差（弧度）。人腿偏离中心
                        最多约 12°，所以给到 25° 左右就够
    """
    if not clusters or ref_dist <= 0.0:
        return None

    span_limit = math.radians(max_span_deg)
    best = None
    for c in clusters:
        if c['n'] > max_points:
            continue
        if c['dist'] > max_range:
            continue
        span = c.get('span')
        if span is not None and span > span_limit:
            continue
        if abs(c['dist'] - ref_dist) > max_dist_diff:
            continue
        if max_angle_diff is not None:
            if abs(wrap_angle(c['angle'] - ref_angle)) > max_angle_diff:
                continue
        if best is None or c['dist'] < best['dist']:
            best = c

    if best is None:
        return None

    return {
        'dist': best['dist'],
        'angle': best['angle'],
        'n': best['n'],
        'kind': 'leg',
        'sep': 0.0,
        'angle_valid': False,
    }


def _score(dist, angle, ref_dist, ref_angle):
    """已锁定时衡量「这个候选像不像上次的目标」。越小越像。

    角度权重给 2 倍：距离相近的两个物体，靠角度区分。
    ref_dist <= 0 表示还没有参考目标（刚丢帧后重新搜索），
    这时退化成「取最近的」。
    """
    if ref_dist <= 0.0:
        return dist
    return abs(dist - ref_dist) + 2.0 * abs(wrap_angle(angle - ref_angle))


def _within_gate(dist, angle, ref_dist, ref_angle,
                 max_jump, max_angle_jump):
    """跳变门限：距离或角度突变得太厉害就不是同一个物体。"""
    if ref_dist <= 0.0:
        return True
    if max_jump is not None and abs(dist - ref_dist) > max_jump:
        return False
    if max_angle_jump is not None:
        if abs(wrap_angle(angle - ref_angle)) > max_angle_jump:
            return False
    return True


def select_follow_target(clusters, locked, ref_dist, ref_angle,
                         lock_max_range, track_max_range,
                         min_points=5, max_jump=None,
                         max_angle_jump=None,
                         pair_enable=True,
                         pair_lock_max_range=1.00,
                         pair_track_max_range=1.20,
                         pair_max_sep=0.60,
                         pair_min_sep=0.05,
                         pair_max_dist_diff=0.35,
                         pair_min_points=2,
                         pair_max_span_deg=45.0,
                         pair_max_lateral_sep=0.50,
                         pair_max_along_sep=0.25,
                         leg_fallback=True,
                         leg_max_points=4,
                         leg_max_dist_diff=0.35):
    """统一的跟随目标选择：**双腿对优先**，其次单簇，最后单腿兜底。

    三条路径并行求候选，最后按「谁更近 / 谁更像上次目标」决胜：

        未锁定：双腿对 / 单簇 → 取距离更近的候选
        已锁定：双腿对 / 单簇 → 取与卡尔曼预测更接近的候选
        两者都没有且已锁定 → 单腿兜底（仅用距离，角度走预测）

    返回 dict：
        {'dist','angle','n','kind','sep','angle_valid'}
        kind 为 'pair' / 'single' / 'leg'
    没有合格目标返回 None。
    """
    if not clusters:
        return None

    # ---------- A. 单簇路径（实体目标：纸箱、椅子、人侧身站）----------
    single = None
    if not locked:
        s = select_target(clusters, False, 0.0, 0.0,
                          lock_max_range, track_max_range,
                          min_points=min_points)
        if s is not None:
            single = {'dist': s[0], 'angle': s[1], 'n': s[2],
                      'kind': 'single', 'sep': 0.0, 'angle_valid': True}
    else:
        s = select_target(clusters, True, ref_dist, ref_angle,
                          lock_max_range, track_max_range,
                          min_points=min_points, max_jump=max_jump)
        if s is not None and _within_gate(s[0], s[1], ref_dist, ref_angle,
                                          max_jump, max_angle_jump):
            single = {'dist': s[0], 'angle': s[1], 'n': s[2],
                      'kind': 'single', 'sep': 0.0, 'angle_valid': True}

    # ---------- B. 双腿对路径（人腿工况）----------
    pair = None
    if pair_enable:
        pairs = find_leg_pairs(
            clusters,
            pair_track_max_range if locked else pair_lock_max_range,
            pair_max_sep, pair_min_sep,
            pair_max_dist_diff, pair_min_points,
            max_span_deg=pair_max_span_deg,
            max_lateral_sep=pair_max_lateral_sep,
            max_along_sep=pair_max_along_sep)

        if locked:
            pairs = [p for p in pairs
                     if _within_gate(p['dist'], p['angle'],
                                     ref_dist, ref_angle,
                                     max_jump, max_angle_jump)]
            if pairs:
                pair = min(pairs, key=lambda p: _score(
                    p['dist'], p['angle'], ref_dist, ref_angle))
        elif pairs:
            pair = pairs[0]        # 已按中点距离升序

        if pair is not None:
            pair['angle_valid'] = True

    # ---------- C. 单腿兜底（只看到一条腿）----------
    if pair is None and single is None and locked and leg_fallback:
        return find_single_leg(clusters, ref_dist, ref_angle,
                               pair_track_max_range,
                               max_points=leg_max_points,
                               max_dist_diff=leg_max_dist_diff,
                               max_angle_diff=max_angle_jump,
                               max_span_deg=pair_max_span_deg)

    # ---------- D. 汇总 ----------
    if pair is None:
        return single
    if single is None:
        return pair

    if not locked:
        return pair if pair['dist'] <= single['dist'] else single

    sp = _score(pair['dist'], pair['angle'], ref_dist, ref_angle)
    ss = _score(single['dist'], single['angle'], ref_dist, ref_angle)
    return pair if sp <= ss else single

