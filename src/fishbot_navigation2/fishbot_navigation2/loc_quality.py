#!/usr/bin/env python3
"""loc_quality.py
============================================================
定位质量评估：激光-地图匹配率 + 位姿稳定性 + σ 参考项

================================================================
为什么需要这个模块（2026-11-01 实机事故复盘）
================================================================
现象：用户说「小车在地图上的位姿有规律乱飘」，周期正好 120 秒。

日志证据（auto_localizer + amcl_tracer）：
    11:47:25  全局定位开始，σ_xy 起手 1.474m
    11:48:35  σ_xy 降到 0.112m  ← 这已经是**正确位姿**
              但收敛线是 0.08m，差 0.032m 判「没收敛」
    11:49:25  120s 超时 → 重新撒粒子 → 位姿全图乱跳 → 循环

根因有两层：

(1) σ（协方差）只反映**粒子云的一致性 precision**，不反映
    **正确性 accuracy**。所有粒子收敛到同一个错误位置，σ 同样很小。
    反过来，正确位姿也可能因为粒子云略散而 σ 超线。
    把 σ 当唯一判据必然两头出错。

(2) 原来的「假收敛」校验 `_check_pose_on_map` 只查**一个 5cm 栅格**：
        row = int((y-oy)/res); val = map[row*width+col]
    这张图 67.9% 是未知区，位姿那一个格子很容易正好落在灰色区域，
    于是永远判「假收敛」→ 永远不收敛 → 120s 重撒 → 乱飘。
    单像素没有冗余，噪声一来就误判。

================================================================
本模块的三件武器
================================================================
A. **激光-地图匹配率**（物理证据，主判据）
   把 471 束激光的端点按当前位姿投影到地图上：
     端点落在障碍物(含膨胀容差) → 命中 hit
     端点落在已知空闲区        → 未命中 miss（激光看到了地图上没有的东西）
     端点落在未知区/越界       → 跳过 unknown（地图没覆盖，无法判断）
   match_rate = hit / (hit + miss)

   这是**471 束激光共同投票**，不是单像素赌博：
     位姿正确           → 80~95%
     位姿偏 20cm        → 30~50%
     粒子云多峰、均值落在墙上 → <20%
   天然识别「σ 很小但位置错」，也天然容忍「σ 略大但位置对」。

B. **位姿稳定性**（把用户说的「看起来收敛了」直接写成代码）
   连续 N 秒内位移 < 5cm 且转角 < 2° → stable。
   比 σ 更贴近人类直觉：地图上的车头图标不动了，就是稳了。

C. **σ 降级为参考项**
   σ 达标不再是一票否决，而是「加分项」：
     σ 达标 + 稳定 + 匹配率≥min_rate          → 收敛
     σ 超线 + 稳定 + 匹配率≥good_rate         → 也收敛 ✅
     σ 超线 + 匹配率低                        → 不收敛，继续盯
   这样 σ=0.112m / 匹配率 85% 那种情况 30 秒内就能放行。

================================================================
安全边界
================================================================
本模块是**纯计算**，不依赖 ROS、不发布任何话题、不控制底盘。
"""

import math
from collections import deque
from dataclasses import dataclass, field

# ============================================================
# 地图栅格取值约定（与 nav2 map_server 的 OccupancyGrid 一致）
# ============================================================
UNKNOWN = -1
OCCUPIED_THRESHOLD = 65        # val >= 65 视为障碍物
FREE_MAX = 65                  # 0 <= val < 65 视为空闲

# 栅格查询结果
CELL_OUTSIDE = 'outside'
CELL_UNKNOWN = 'unknown'
CELL_OCCUPIED = 'occupied'
CELL_FREE = 'free'


# ============================================================
# 一、地图栅格（带膨胀）
# ============================================================
class MapGrid:
    """OccupancyGrid 的快速查询封装。

    关键点：**不做 Y 翻转**。
    /map（OccupancyGrid）的 data 行号已经是「世界坐标系自下往上」的顺序
    （map_server 读 PGM 时已翻过一次），所以
        row = int((y - origin_y) / res)
    就是对的。本项目旧代码又翻了一次，导致读到上下镜像的像素 ——
    详见 auto_localizer._check_pose_on_map 的注释。

    Args:
        data: OccupancyGrid.data（list[int]）
        width, height: 栅格尺寸
        res: 每格多少米
        origin_x, origin_y: 地图左下角的世界坐标
        inflate_cells: 障碍物膨胀半径（格）。激光端点落在这个范围内
            就算命中 —— 容忍地图离散化误差与定位小偏差。
    """

    __slots__ = ('width', 'height', 'res', 'origin_x', 'origin_y',
                 '_known', '_occ', '_dil', 'inflate_cells')

    def __init__(self, data, width, height, res, origin_x, origin_y,
                 inflate_cells=2):
        self.width = int(width)
        self.height = int(height)
        self.res = float(res)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.inflate_cells = max(0, int(inflate_cells))

        n = self.width * self.height
        known = bytearray(n)
        occ = bytearray(n)
        for i in range(min(n, len(data))):
            v = data[i]
            if v is None or v < 0:
                continue
            known[i] = 1
            if v >= OCCUPIED_THRESHOLD:
                occ[i] = 1
        self._known = known
        self._occ = occ
        self._dil = self._dilate(occ) if self.inflate_cells else occ

    def _dilate(self, occ):
        """把障碍物向外膨胀 inflate_cells 格（方形结构元）。"""
        w, h, r = self.width, self.height, self.inflate_cells
        out = bytearray(len(occ))
        offsets = [(dx, dy)
                   for dy in range(-r, r + 1)
                   for dx in range(-r, r + 1)]
        for i, v in enumerate(occ):
            if not v:
                continue
            col = i % w
            row = i // w
            for dx, dy in offsets:
                c2 = col + dx
                r2 = row + dy
                if 0 <= c2 < w and 0 <= r2 < h:
                    out[r2 * w + c2] = 1
        return out

    # --------------------------------------------------------
    def world_to_cell(self, x, y):
        """世界坐标 → (col, row)；越界返回 None。"""
        col = int((x - self.origin_x) / self.res)
        row = int((y - self.origin_y) / self.res)
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None

    def cell_kind(self, col, row, inflated=True):
        """返回栅格类别（四选一）。"""
        idx = row * self.width + col
        if not self._known[idx]:
            return CELL_UNKNOWN
        if (self._dil if inflated else self._occ)[idx]:
            return CELL_OCCUPIED
        return CELL_FREE

    def truth(self, x, y, inflated=True):
        """世界坐标 → 栅格类别。"""
        cell = self.world_to_cell(x, y)
        if cell is None:
            return CELL_OUTSIDE
        return self.cell_kind(cell[0], cell[1], inflated)


# ============================================================
# 二、激光-地图匹配率
# ============================================================
@dataclass
class MatchResult:
    """一次匹配的统计结果。"""
    hit: int = 0
    miss: int = 0
    unknown: int = 0
    skipped: int = 0
    rate: float = None          # hit/(hit+miss)；样本不足时 None
    coverage: float = 0.0       # (hit+miss)/有效束，衡量这个 rate 可信度
    total: int = 0

    @property
    def reliable(self) -> bool:
        return self.rate is not None

    def as_dict(self):
        return {
            'hit': self.hit, 'miss': self.miss, 'unknown': self.unknown,
            'skipped': self.skipped, 'total': self.total,
            'match_rate': (None if self.rate is None
                           else round(self.rate, 3)),
            'coverage': round(self.coverage, 3),
        }


class LaserMapMatcher:
    """把激光端点投影到地图上，统计命中率。

    Args:
        min_samples: hit+miss 少于此数就认为统计不可信（rate=None）
        min_coverage: (hit+miss)/有效束 低于此值认为 rate 不可信
        stride: 抽帧步长。471 束全算也就几百次栅格查找，
            2Hz 下毫无压力，默认 1 全用。
        trace_step_cells: 端点落在未知/地图外时，沿射线回溯的步长（格）
        trace_tol_m: 地图上的墙与激光实测距离差在多少米内算「一致」
    """

    def __init__(self, min_samples=40, min_coverage=0.25, stride=1,
                 trace_step_cells=2, trace_tol_m=0.15):
        self.min_samples = int(min_samples)
        self.min_coverage = float(min_coverage)
        self.stride = max(1, int(stride))
        self.trace_step_cells = max(1, int(trace_step_cells))
        self.trace_tol_m = float(trace_tol_m)
        self._last = MatchResult()

    # --------------------------------------------------------
    def _trace_ray(self, grid, ox, oy, ca, sa, r):
        """端点不明确时，沿射线找最近的已知占据格。

        返回 'hit' / 'miss' / 'unknown'。

        为什么需要（本项目的实际情况）：
          事故地图 126x207 里 **67.9% 是未知区**。小车多半站在一小块
          已知空闲区的中间，射出去的激光端点很容易落在未知区，
          按端点判定只能把它们全部丢掉 —— 实测覆盖率会掉到 20% 以下，
          匹配率直接失效。

          但射线**途经**的格子很多是已知的：穿过的空闲区说明
          "这一段确实没东西"，撞到的墙说明"这里应该有东西"。
          所以沿射线走一遍，比较「地图上最近墙的距离」与
          「激光实测距离」：
             相差 ≤ trace_tol_m  → 一致，命中
             相差更大           → 地图与实测矛盾，未命中
             整条射线都在未知区 → 无从判断，跳过
        """
        step = grid.res * self.trace_step_cells
        d = step
        saw_known = False
        limit = r + step
        while d <= limit:
            kind = grid.truth(ox + d * ca, oy + d * sa)
            if kind is CELL_OCCUPIED:
                return 'hit' if abs(r - d) <= self.trace_tol_m else 'miss'
            if kind is CELL_FREE:
                saw_known = True
            elif kind is CELL_OUTSIDE:
                break            # 走出地图，后面的都无从判断
            d += step
        # 走完没撞到墙：只有确认过空闲区才敢说"矛盾"
        return 'miss' if saw_known else 'unknown'

    # --------------------------------------------------------
    def evaluate(self, grid, pose, ranges,
                 angle_min=0.0, angle_increment=0.01,
                 range_min=0.0, range_max=100.0,
                 laser_offset_x=0.0, laser_offset_y=0.0) -> MatchResult:
        """计算匹配率。

        Args:
            grid: MapGrid
            pose: (x, y, yaw)
            ranges: LaserScan.ranges
            angle_min / angle_increment: 激光角度定义
            range_min / range_max: 量程
            laser_offset_x/y: 激光相对 base_footprint 的平面偏移。
                本项目 URDF 里 laser_joint 的 xyz="0 0 0.075"，
                x/y 都是 0，所以默认 0 即可（2D 投影用不到 z）。

        Returns:
            MatchResult
        """
        res = MatchResult()
        if grid is None or pose is None or not ranges:
            return res

        res.total = len(ranges)
        x0, y0, yaw0 = pose
        cos_y, sin_y = math.cos(yaw0), math.sin(yaw0)
        # 激光中心在这个位姿下的世界坐标
        ox = x0 + laser_offset_x * cos_y - laser_offset_y * sin_y
        oy = y0 + laser_offset_x * sin_y + laser_offset_y * cos_y

        # 端点几乎贴到 max_range 的束通常是「没打到东西」，不算数
        hit_max = range_max - max(1e-3, range_max * 1e-3)
        step = self.stride
        # 光束打到地图边界外时的回退比例。
        #
        # 为什么需要：地图最外圈的墙就在栅格 col=0 / col=width-1 上，
        # 而墙的外表面正好落在 x = origin_x + width*res 这条边界线上。
        # 激光端点若精确落在边界（或因为浮点误差越过一点点），
        # world_to_cell 会判越界 → 白白记成 unknown。
        # 实测（4m×4m 合成房间）180 束里有 90 束因此被丢弃，
        # coverage 只有 50%。沿着射线往回退一点再查即可解决。
        fallbacks = (0.98, 0.95, 0.90)

        for i in range(0, len(ranges), step):
            r = ranges[i]
            # NaN / inf 是小车的软肋，必须显式排除
            if r is None:
                res.skipped += 1
                continue
            try:
                r = float(r)
            except (TypeError, ValueError):
                res.skipped += 1
                continue
            if not math.isfinite(r) or r < range_min or r >= hit_max:
                res.skipped += 1
                continue

            a = yaw0 + angle_min + i * angle_increment
            ca, sa = math.cos(a), math.sin(a)

            kind = grid.truth(ox + r * ca, oy + r * sa)
            if kind is CELL_OUTSIDE:
                for k in fallbacks:
                    kind = grid.truth(ox + r * k * ca, oy + r * k * sa)
                    if kind is not CELL_OUTSIDE:
                        break

            if kind is CELL_OCCUPIED:
                verdict = 'hit'
            elif kind is CELL_FREE:
                verdict = 'miss'
            else:
                # 端点落在未知区/地图外 —— 退化为沿射线判断
                verdict = self._trace_ray(grid, ox, oy, ca, sa, r)

            if verdict == 'hit':
                res.hit += 1
            elif verdict == 'miss':
                res.miss += 1
            else:
                res.unknown += 1

        conclusive = res.hit + res.miss
        valid = conclusive + res.unknown
        if valid > 0:
            res.coverage = conclusive / float(valid)
        if conclusive >= self.min_samples and res.coverage >= self.min_coverage:
            res.rate = res.hit / float(conclusive)

        self._last = res
        return res

    @property
    def last(self):
        return self._last


# ============================================================
# 三、位姿稳定性（"看起来收敛了"的代码化）
# ============================================================
class PoseStabilityTracker:
    """判断位姿在最近 window_sec 内是否几乎没动。

    Args:
        window_sec: 观察窗口
        move_tol: 允许的最大位移（米）
        rot_tol_deg: 允许的最大转角（度）

    与 σ 的区别：σ 描述"粒子散不散"，本类描述"估计值稳不稳"。
    用户在地图上看车头图标是否乱动，看的其实就是这个。
    """

    def __init__(self, window_sec=3.0, move_tol=0.05, rot_tol_deg=2.0):
        self.window_sec = max(0.2, float(window_sec))
        self.move_tol = max(1e-3, float(move_tol))
        self.rot_tol = math.radians(max(0.1, float(rot_tol_deg)))
        self._buf = deque()          # (t, x, y, yaw)
        self._stable_since = None
        self._last = (False, 0.0)

    def reset(self):
        self._buf.clear()
        self._stable_since = None
        self._last = (False, 0.0)

    @property
    def last(self):
        """最近一次 update() 的结果 (stable, stable_sec)。

        供状态发布之类「不能再喂样本」的场景使用 —— 重复 update 会往
        窗口里塞重复样本，把 stable_sec 算长。
        """
        return self._last

    @staticmethod
    def _ang_delta(a, b):
        """归一化到 (-pi, pi]。"""
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d <= -math.pi:
            d += 2.0 * math.pi
        return d

    def update(self, t, pose):
        """喂一个位姿采样，返回 (stable, stable_sec)。"""
        if pose is None:
            return self._last

        self._buf.append((float(t), float(pose[0]), float(pose[1]),
                          float(pose[2])))
        cutoff = float(t) - self.window_sec

        # ★ 裁剪时必须**多留一个**边界样本。
        #
        # 原来的写法是 `while self._buf[0][0] < cutoff: popleft()`，
        # 而采样周期正好是 0.5s、窗口 3.0s —— 最老的样本恰好落在边界上。
        # 计数器一有微小抖动（create_timer 本来就不保证准时），
        # 那个样本就会被裁掉，窗口实际只剩 2.5s，
        # 于是 `span < window*0.99` 永远成立，**永远判不出稳定**。
        # 实测现象：位姿明明一动不动，stable 一直是 False，
        # 位姿稳定性这条判据等于失效。
        #
        # 改成「第二个样本都过期了才丢第一个」，保证裁剪后
        # span >= window_sec。
        while len(self._buf) > 2 and self._buf[1][0] <= cutoff:
            self._buf.popleft()

        # 窗口还没攒满 → 不算稳定（避免"刚开始就宣布稳定"）
        span = self._buf[-1][0] - self._buf[0][0]
        if span < self.window_sec * 0.99:
            self._stable_since = None
            self._last = (False, span)
            return self._last

        _, x0, y0, yaw0 = self._buf[0]
        worst_move = 0.0
        worst_rot = 0.0
        for _, x, y, yaw in self._buf:
            d = math.hypot(x - x0, y - y0)
            if d > worst_move:
                worst_move = d
            r = abs(self._ang_delta(yaw, yaw0))
            if r > worst_rot:
                worst_rot = r

        if worst_move > self.move_tol or worst_rot > self.rot_tol:
            self._stable_since = None
            self._last = (False, 0.0)
            return self._last

        if self._stable_since is None:
            self._stable_since = float(t)
        self._last = (True, float(t) - self._stable_since)
        return self._last


# ============================================================
# 四、综合判定
# ============================================================
@dataclass
class Decision:
    """一次收敛判定的结果。"""
    converged: bool = False
    veto: bool = False          # 物理证据明确否决（匹配率太低）
    progress: float = 0.0       # 0~1，给网页进度条用
    reason: str = ''
    hint: str = ''
    # 参与判定的原始量，便于日志/前端展示
    sigma_ok: bool = False
    match_rate: float = None
    match_ok: bool = False
    stable: bool = False
    stable_sec: float = 0.0

    def as_dict(self):
        return {
            'converged': self.converged,
            'veto': self.veto,
            'progress': round(self.progress, 3),
            'reason': self.reason,
            'hint': self.hint,
            'sigma_ok': self.sigma_ok,
            'match_rate': (None if self.match_rate is None
                           else round(self.match_rate, 3)),
            'match_ok': self.match_ok,
            'stable': self.stable,
            'stable_sec': round(self.stable_sec, 2),
        }


class QualityAssessor:
    """把 σ / 匹配率 / 稳定性综合成一个收敛判定。

    判定顺序（物理证据优先，σ 只做加分）：

        1. 匹配率可信 且 < bad_rate        → 否决（粒子云位置是错的）
        2. σ 达标 且 稳定 且 匹配率≥min_rate → 收敛
           （匹配率不可信时退化为「σ 达标 且 稳定」——无激光也能用）
        3. σ 超线 但 稳定 且 匹配率≥good_rate → 收敛 ★ 本次事故的关键
        4. 其它                              → 继续等

    阈值标定依据（4m×4m 合成房间 + 内墙 + 未知区，471 束激光实测）：
        正确位姿                → 1.000
        平移偏 0.1m             → 0.996
        平移偏 0.2m             → 0.501
        旋转偏 3°               → 0.981
        旋转偏 8°               → 0.781
        旋转偏 15°              → 0.569
        旋转偏 30°              → 0.308
    所以 good_rate=0.75 恰好卡在「8° 以内」，
    min_rate=0.55 卡在「15° 以内」，bad_rate=0.40 否决 30° 以上。

    Args:
        cov_xy / cov_yaw: σ 门限（米 / 弧度），沿用原有 converge_cov_*
        good_rate: 匹配率达到这个值 + 稳定 → 即便 σ 超线也放行
        min_rate: σ 达标时，匹配率至少要这么高
        bad_rate: 匹配率低于此值 → 一票否决，无论 σ 多小
    """

    def __init__(self, cov_xy=0.08, cov_yaw=0.05,
                 good_rate=0.75, min_rate=0.55, bad_rate=0.40):
        self.cov_xy = float(cov_xy)
        self.cov_yaw = float(cov_yaw)
        self.good_rate = float(good_rate)
        self.min_rate = float(min_rate)
        self.bad_rate = float(bad_rate)

    # --------------------------------------------------------
    def assess(self, std_xy, std_yaw, match: MatchResult,
               stable: bool, stable_sec: float) -> Decision:
        d = Decision(stable=stable, stable_sec=stable_sec)
        if std_xy is None or std_yaw is None:
            d.reason = '尚未收到 /amcl_pose'
            return d

        sigma_ok = (std_xy <= self.cov_xy) and (std_yaw <= self.cov_yaw)
        d.sigma_ok = sigma_ok

        rate = match.rate if match is not None else None
        d.match_rate = rate
        reliable = rate is not None

        # ---- 1. 物理否决 ----
        if reliable and rate < self.bad_rate:
            d.veto = True
            d.reason = ('激光与地图严重不符（匹配率 %.0f%%），'
                        '当前位姿是错的' % (rate * 100))
            d.hint = ('小车当前位置的估计与实际不符。请缓慢移动或原地转动'
                      '小车，让 AMCL 重新收敛后再下指令。')
            d.match_ok = False
            d.progress = self._progress(std_xy, rate, stable, stable_sec,
                                        False)
            return d

        if reliable and rate >= self.good_rate:
            d.match_ok = True
        elif reliable and rate >= self.min_rate:
            d.match_ok = True
        else:
            d.match_ok = False

        # ---- 2. σ 达标路线 ----
        if sigma_ok and stable and (not reliable or rate >= self.min_rate):
            d.converged = True
            d.progress = 1.0
            d.reason = ('σ 达标（%.3fm/%.1f°）且位姿稳定 %.1fs'
                        % (std_xy, math.degrees(std_yaw), stable_sec))
            if reliable:
                d.reason += '，激光匹配 %.0f%%' % (rate * 100)
            return d

        # ---- 3. σ 超线但物理证据充分 ----
        if reliable and rate >= self.good_rate and stable:
            d.converged = True
            d.progress = 1.0
            d.reason = ('σ 略大（%.3fm/%.1f°）但激光匹配 %.0f%% 且位姿稳定'
                        ' %.1fs —— 判定可信'
                        % (std_xy, math.degrees(std_yaw), rate * 100,
                           stable_sec))
            return d

        # ---- 4. 继续等 ----
        if not stable:
            d.reason = '位姿还在动，等它稳下来'
            d.hint = '缓慢原地转动小车有助于 AMCL 消除角度歧义'
        elif reliable and rate < self.min_rate:
            d.reason = ('激光匹配率偏低（%.0f%%），当前位姿可能不对'
                        % (rate * 100))
            d.hint = '缓慢移动或转动小车，让激光与地图重新对齐'
        elif not sigma_ok:
            d.reason = ('粒子云还不够集中（σ %.3fm/%.1f°，门限 %.3fm/%.1f°）'
                        % (std_xy, math.degrees(std_yaw),
                           self.cov_xy, math.degrees(self.cov_yaw)))
            d.hint = '静止时 AMCL 收敛很慢，转动一下小车会快很多'
        else:
            d.reason = '还在等待更多观测确认'

        d.progress = self._progress(std_xy, rate, stable, stable_sec, False)
        return d

    # --------------------------------------------------------
    def _progress(self, std_xy, rate, stable, stable_sec, converged):
        if converged:
            return 1.0
        # σ 分项：到门限 4 倍以外记 0
        span = max(1e-6, self.cov_xy * 4.0)
        p_sigma = 1.0 - min(1.0, std_xy / span)

        if rate is None:
            p_match = p_sigma
        else:
            lo, hi = self.bad_rate, self.good_rate
            p_match = (rate - lo) / (hi - lo) if hi > lo else 0.0
            p_match = max(0.0, min(1.0, p_match))

        p_stable = 1.0 if stable else min(1.0, stable_sec / 3.0)
        val = 0.30 * p_sigma + 0.50 * p_match + 0.20 * p_stable
        return max(0.0, min(1.0, val))


# ============================================================
# 五、状态快照（供 /localization_status 话题发布）
# ============================================================
@dataclass
class StatusSnapshot:
    state: str = 'unknown'
    ready: bool = False
    progress: float = 0.0
    reason: str = ''
    hint: str = ''
    # σ
    std_xy: float = None
    std_yaw_deg: float = None
    sigma_ok: bool = False
    # 匹配
    match_rate: float = None
    match_ok: bool = False
    hit: int = 0
    miss: int = 0
    unknown: int = 0
    coverage: float = 0.0
    # 稳定性
    stable: bool = False
    stable_sec: float = 0.0
    # 位姿
    pose: dict = field(default_factory=dict)
    # 辅助
    scan_ok: bool = False
    extra: dict = field(default_factory=dict)

    def as_dict(self):
        d = {
            'state': self.state,
            'ready': bool(self.ready),
            'progress': round(float(self.progress), 3),
            'reason': self.reason,
            'hint': self.hint,
            'std_xy': (None if self.std_xy is None
                       else round(self.std_xy, 4)),
            'std_yaw_deg': (None if self.std_yaw_deg is None
                            else round(self.std_yaw_deg, 2)),
            'sigma_ok': bool(self.sigma_ok),
            'match_rate': (None if self.match_rate is None
                           else round(self.match_rate, 3)),
            'match_ok': bool(self.match_ok),
            'hit': self.hit,
            'miss': self.miss,
            'unknown': self.unknown,
            'coverage': round(float(self.coverage), 3),
            'stable': bool(self.stable),
            'stable_sec': round(float(self.stable_sec), 2),
            'pose': self.pose,
            'scan_ok': bool(self.scan_ok),
        }
        if self.extra:
            d.update(self.extra)
        return d
