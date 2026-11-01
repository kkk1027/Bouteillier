#!/usr/bin/env python3
"""test_loc_quality.py
============================================================
定位质量评估（匹配率 + 稳定性 + σ 降级）的回归测试

背景（2026-11-01 实机事故）：
    用户说「小车在地图上的位姿有规律乱飘」。
    实测 σ_xy 最好只到 0.112m，而收敛线是 0.08m —— 差 0.032m
    就判「没收敛」，120s 一到重撒粒子，位姿全图乱跳，无限循环。
    更糟的是「假收敛」校验只查一个 5cm 栅格，而地图 67.9% 是未知区，
    正确位姿被判「假收敛」的概率极高。

本测试不需要 ROS、不需要雷达、不接触硬件。
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from fishbot_navigation2.loc_quality import (      # noqa: E402
    CELL_FREE, CELL_OCCUPIED, CELL_OUTSIDE, CELL_UNKNOWN,
    Decision, LaserMapMatcher, MapGrid, MatchResult,
    PoseStabilityTracker, QualityAssessor, StatusSnapshot,
)

ok = 0
total = 0


def check(name, cond, extra=''):
    global ok, total
    total += 1
    ok += bool(cond)
    print('  %s %s %s' % ('✅' if cond else '❌', name, extra))


# ============================================================
# 造图：4m x 4m 房间 + 中间一堵竖墙 + 右下角未知区
# ------------------------------------------------------------
# 房间四周墙 → 提供定位信息
# 中间竖墙   → 打破对称，让平移误差可分辨
# 未知区     → 复刻事故地图"大片没扫到"的特点
# ============================================================
RES, W, H = 0.05, 80, 80
_map = [0] * (W * H)
for _i in range(W):
    _map[_i] = 100
    _map[(H - 1) * W + _i] = 100
for _j in range(H):
    _map[_j * W] = 100
    _map[_j * W + W - 1] = 100
for _j in range(20, 60):
    _map[_j * W + 40] = 100        # 中间竖墙 col=40
for _j in range(60, 80):
    for _i in range(50, 80):
        _map[_j * W + _i] = -1     # 右下角未知

GRID = MapGrid(_map, W, H, RES, 0.0, 0.0, inflate_cells=2)
NBEAM, AMIN, AINC, RMAX = 471, -math.pi, 2 * math.pi / 471, 12.0


def simulate_ranges(x, y, yaw):
    """按栅格真实地"打一帧激光"：遇到占据格就停。"""
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
            if _map[row * W + col] >= 65:
                hit = d
                break
            d += RES * 0.5
        out.append(hit)
    return out


MATCHER = LaserMapMatcher(min_samples=20, min_coverage=0.2)


def match_at(pose, real_pose):
    return MATCHER.evaluate(
        GRID, pose, simulate_ranges(*real_pose), AMIN, AINC, 0.05, RMAX)


# ============================================================
print('一、地图栅格索引（不许 Y 翻转）')
# ============================================================
check('左下角是墙', GRID.cell_kind(0, 0) is CELL_OCCUPIED)
# 右上角被“未知区”覆盖了（复刻事故地图哪哪都是灰的特点），
# 改查右侧墙的靠上位置
check('右侧墙是墙', GRID.cell_kind(W - 1, 20) is CELL_OCCUPIED)
check('上侧墙是墙', GRID.cell_kind(20, H - 1) is CELL_OCCUPIED)
check('中心是空闲', GRID.cell_kind(20, 20) is CELL_FREE)
check('右下角未知', GRID.cell_kind(70, 70) is CELL_UNKNOWN)
check('地图外', GRID.truth(-1.0, 2.0) is CELL_OUTSIDE)
# 关键回归：row = int((y-oy)/res)，绝不能是 height-1-...
check('世界坐标 (0.1, 3.9) → 左上角附近（y 向上）',
      GRID.world_to_cell(0.1, 3.9) == (2, 78))
check('世界坐标 (0.1, 0.1) → 左下角附近（y 向下）',
      GRID.world_to_cell(0.1, 0.1) == (2, 2))

# ============================================================
print()
print('二、激光-地图匹配率（A）')
# ============================================================
correct = (1.0, 1.0, 0.0)
r = match_at(correct, correct)
check('正确位姿匹配率 ≈ 100%', r.rate is not None and r.rate > 0.98,
      '%.3f' % (r.rate or -1))
check('正确位姿覆盖率高', r.coverage > 0.9, '%.2f' % r.coverage)

# 平移误差
rates = {}
for off in (0.0, 0.1, 0.2, 0.4):
    rr = match_at((1.0 + off, 1.0, 0.0), correct)
    rates[off] = rr.rate
    print('     平移 %.1fm → %.3f' % (off, rr.rate or -1))
check('平移 0.1m 仍在高位', rates[0.1] > 0.95)
check('平移 0.2m 明显下滑', rates[0.2] < 0.70,
      '%.3f' % (rates[0.2] or -1))
check('平移误差单调下降', rates[0.0] > rates[0.1] > rates[0.2])

# 旋转误差
rot = {}
for deg in (0, 3, 8, 15, 30):
    rr = match_at((1.0, 1.0, math.radians(deg)), correct)
    rot[deg] = rr.rate
    print('     旋转 %3d° → %.3f' % (deg, rr.rate or -1))
check('旋转 3° 仍在高位', rot[3] > 0.95)
check('旋转 15° 已明显不符', rot[15] < 0.70,
      '%.3f' % (rot[15] or -1))
check('旋转误差单调下降',
      rot[0] > rot[3] > rot[8] > rot[15] > rot[30])

# 恶劣输入
check('ranges 为空 → rate=None',
      MATCHER.evaluate(GRID, correct, [], AMIN, AINC, 0.05, RMAX).rate is None)
check('grid 为 None → rate=None',
      MATCHER.evaluate(None, correct, [1.0] * 100, AMIN, AINC, 0.05,
                       RMAX).rate is None)
nan_ranges = [float('nan')] * 10 + [float('inf')] * 10 + [None] * 10
res_nan = MATCHER.evaluate(GRID, correct, nan_ranges, AMIN, AINC, 0.05, RMAX)
check('NaN/inf/None 被安全跳过', res_nan.rate is None
      and res_nan.skipped == 30)
check('样本太少 → rate=None（不瞎下结论）',
      MATCHER.evaluate(GRID, correct, [1.0] * 5, AMIN, 0.01, 0.05,
                       RMAX).rate is None)

# ============================================================
print()
print('三、位姿稳定性（B）')
# ============================================================
trk = PoseStabilityTracker(window_sec=3.0, move_tol=0.05, rot_tol_deg=2.0)
for k in range(20):
    st, sec = trk.update(k * 0.5, (1.0, 1.0, 0.0))
check('位姿不动 → 稳定', st is True, '%.1fs' % sec)
check('稳定时长随窗口增长', sec >= 3.0)

trk = PoseStabilityTracker(window_sec=3.0, move_tol=0.05, rot_tol_deg=2.0)
for k in range(20):
    st, _ = trk.update(k * 0.5, (1.0 + k * 0.02, 1.0, 0.0))   # 持续漂
check('位姿持续漂移 → 不稳定', st is False)

trk = PoseStabilityTracker(window_sec=3.0, move_tol=0.05, rot_tol_deg=2.0)
for k in range(20):
    st, _ = trk.update(k * 0.5, (1.0, 1.0, math.radians(k * 3.0)))  # 持续转
check('位姿持续转动 → 不稳定', st is False)

# 角度环绕：359° → 1° 只差 2°，不该判成 358°
trk = PoseStabilityTracker(window_sec=2.0, move_tol=1.0, rot_tol_deg=5.0)
trk.update(0.0, (1.0, 1.0, math.radians(179.0)))
trk.update(1.0, (1.0, 1.0, math.radians(-179.0)))
st, _ = trk.update(2.0, (1.0, 1.0, math.radians(179.0)))
check('±180° 环绕正确处理（2° 差异不算不稳）', st is True)

trk = PoseStabilityTracker(window_sec=3.0)
trk.update(0.0, (1.0, 1.0, 0.0))
st, _ = trk.update(0.5, (1.0, 1.0, 0.0))
check('窗口没攒满不算稳定（避免刚开始就宣布好了）', st is False)

# ============================================================
print()
print('四、综合判定（C：σ 降级为参考项）')
# ============================================================
A = QualityAssessor(cov_xy=0.08, cov_yaw=0.05)


def mk(rate, unk=40, conclusive=400):
    """构造一个指定匹配率的 MatchResult。

    ★ 早期版本这里写错了：不管传什么 rate 都返回 300/360=83%，
      导致四条断言假通过。现在按 rate 反推 hit/miss。
    """
    m = MatchResult()
    if rate is None:
        m.hit = m.miss = 0
        m.unknown = unk
        m.total = unk
        m.coverage = 0.0
        m.rate = None
        return m
    m.hit = int(round(conclusive * rate))
    m.miss = conclusive - m.hit
    m.unknown = unk
    m.total = m.hit + m.miss + m.unknown
    m.coverage = conclusive / float(m.total)
    m.rate = m.hit / float(conclusive)
    return m


# --- 事故里那个关键场景：σ=0.112m 超线，但匹配率高 + 稳定 ---
d = A.assess(0.112, 0.02, mk(0.90), True, 5.0)
check('★ σ=0.112m 超线 + 匹配 90% + 稳定 → 判定收敛（事故主场景）',
      d.converged, d.reason)

# --- σ 达标 + 匹配率高 + 稳定 ---
d = A.assess(0.03, 0.02, mk(0.90), True, 5.0)
check('σ 达标 + 匹配高 + 稳定 → 收敛', d.converged, d.reason)

# --- σ 达标 + 匹配率高但不稳定 ---
d = A.assess(0.03, 0.02, mk(0.90), False, 0.0)
check('位姿还在动 → 不收敛', not d.converged, d.reason)

# --- σ 达标但匹配率低（假收敛） ---
d = A.assess(0.02, 0.01, mk(0.50), True, 5.0)
check('σ 很小但匹配率不足 → 不收敛（识别假收敛）',
      not d.converged, d.reason)

# --- 匹配率极低 → 物理否决 ---
d = A.assess(0.02, 0.01, mk(0.20), True, 5.0)
check('匹配率 20% → 一票否决', d.veto and not d.converged, d.reason)
check('否决时给出可操作建议', bool(d.hint))

# --- σ 大 + 匹配率中等 + 稳定 → 不该放行 ---
d = A.assess(0.40, 0.20, mk(0.60), True, 5.0)
check('σ 大 + 匹配一般 → 不放行', not d.converged, d.reason)

# --- 无激光时退化为只看 σ（不能把功能完全锁死） ---
d = A.assess(0.03, 0.02, mk(None), True, 5.0)
check('无匹配数据时退化为 σ + 稳定 → 收敛', d.converged, d.reason)
d = A.assess(0.30, 0.02, mk(None), True, 5.0)
check('无匹配数据且 σ 大 → 不收敛', not d.converged, d.reason)

# --- 进度条 ---
d = A.assess(0.03, 0.02, mk(0.90), True, 5.0)
check('收敛时 progress=1.0', d.progress == 1.0)
d = A.assess(0.60, 0.30, mk(0.45), False, 0.0)
check('很差时 progress 很低', d.progress < 0.4, '%.2f' % d.progress)
d1 = A.assess(0.40, 0.20, mk(0.55), False, 0.0)
d2 = A.assess(0.10, 0.05, mk(0.70), True, 4.0)
check('progress 随质量上升单调变好', d2.progress > d1.progress,
      '%.2f > %.2f' % (d2.progress, d1.progress))

# ============================================================
print()
print('五、状态快照（E：/localization_status）')
# ============================================================
snap = StatusSnapshot(
    state='converging', ready=False, progress=0.62,
    reason='位姿还在动', hint='转一下小车',
    std_xy=0.1123, std_yaw_deg=4.52, sigma_ok=False,
    match_rate=0.903, match_ok=True,
    hit=310, miss=33, unknown=128, coverage=0.727,
    stable=True, stable_sec=2.4,
    pose={'x': 1.234, 'y': 0.567, 'yaw_deg': 13.5},
    scan_ok=True,
)
d = snap.as_dict()
check('as_dict 含 ready', d['ready'] is False)
check('as_dict 含 match_rate', abs(d['match_rate'] - 0.903) < 1e-6)
check('as_dict 含 progress', abs(d['progress'] - 0.62) < 1e-6)
check('as_dict 含 reason/hint', d['reason'] and d['hint'])
import json                                     # noqa: E402
try:
    json.dumps(d)
    jok = True
except Exception:                               # noqa: BLE001
    jok = False
check('as_dict 可 JSON 序列化', jok)
check('None 值不炸（无激光时）',
      json.loads(json.dumps(StatusSnapshot().as_dict()))['match_rate']
      is None)

# ============================================================
print()
print('六、事故场景端到端复现')
# ------------------------------------------------------------
# 完全照抄事故日志的数字，看新逻辑会不会再卡 120 秒
# ------------------------------------------------------------
print('    事故日志里的采样（auto_localizer 原始输出）：')
accident = [
    (10, 1.499, 123.5, 0.35),
    (21, 1.026, 50.7, 0.42),
    (31, 0.687, 33.8, 0.50),
    (41, 0.349, 19.6, 0.58),
    (51, 0.257, 16.4, 0.66),
    (62, 0.305, 20.5, 0.70),
    (72, 0.112, 5.2, 0.88),      # ← 最好的一次，位姿其实是对的
    (82, 0.265, 16.8, 0.74),
    (92, 0.136, 8.3, 0.85),
    (103, 0.477, 30.0, 0.62),
    (113, 1.692, 75.6, 0.35),
]
fired_at = None
for t, sxy, syaw_deg, rate in accident:
    dd = A.assess(sxy, math.radians(syaw_deg), mk(rate), True, 4.0)
    print('    %3ds  σ=%.3fm 匹配=%.0f%%  →  %s'
          % (t, sxy, rate * 100,
             '✅ 收敛' if dd.converged else
             ('⛔ 否决' if dd.veto else '…等待')))
    if dd.converged and fired_at is None:
        fired_at = t

check('★ 事故里 72s 那次（σ=0.112m 匹配 88%）就能判收敛',
      fired_at is not None and fired_at <= 72, '第 %s 秒' % fired_at)
check('不再拖到 120s 重撒', fired_at is not None and fired_at < 120)

print()
print('=' * 60)
print('通过 %d / %d' % (ok, total))
print('=' * 60)
sys.exit(0 if ok == total else 1)
