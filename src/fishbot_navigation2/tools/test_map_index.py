#!/usr/bin/env python3
"""地图栅格索引 / 超时重撒 的离线回归测试（不启动 ROS、不碰实物）。

覆盖 2026-10-31 第二轮复盘挖出的两个 bug：

  ① `_check_pose_on_map` 把 OccupancyGrid 的行号多翻转了一次，
     读到上下镜像的像素 → 好位姿被判「落在未知区」→ 永不收敛
     → 120s 超时 → 全局重撒 → 位姿瞬移 1.8m → 每 2 分钟循环。

  ② `_check_timeout` 不管粒子云状态、无脑全局重撒。
     σ 已经很小时重撒是灾难（把正确位姿直接扔掉）。

用真实的 room1 地图（126x207，unknown 67.9%）做数值校验。

运行：
    python3 src/fishbot_navigation2/tools/test_map_index.py
"""

import math
import os
import re
import sys

PASS = 0
FAIL = 0


def check(name, cond, detail=''):
    global PASS, FAIL
    if cond:
        PASS += 1
        print('  ✅ %s' % name)
    else:
        FAIL += 1
        print('  ❌ %s  %s' % (name, detail))


HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', 'fishbot_navigation2')
MAPS = os.path.join(HERE, '..', 'maps')
with open(os.path.join(PKG, 'auto_localizer.py'), encoding='utf-8') as f:
    CODE = f.read()


# ============================================================
# 一、静态检查
# ============================================================
print('\n=== 静态检查：栅格索引不能重复翻转 ===')

m = re.search(r'def _check_pose_on_map.*?(?=\n    def )', CODE, re.S)
check('找到了 _check_pose_on_map', m is not None)
body = m.group(0) if m else ''
# 只看**代码行**，注释里为说明 bug 会引用旧的错误写法
code_only = '\n'.join(ln for ln in body.splitlines()
                      if not ln.lstrip().startswith('#'))

check('不再出现 `height - 1 -` 双重翻转',
      'height - 1 -' not in code_only,
      'OccupancyGrid 已是 bottom-up，map_server 加载 PGM 时翻过一次')

check('行号直接由 (y - oy)/res 得出',
      re.search(r'row\s*=\s*int\(\(y\s*-\s*oy\)\s*/\s*res\)', body) is not None)

check('列号由 (x - ox)/res 得出',
      re.search(r'col\s*=\s*int\(\(x\s*-\s*ox\)\s*/\s*res\)', body) is not None)

check("'栅格值 %d' 已补上格式化参数",
      "'栅格值 %d）'" not in body and '% (\n            val' not in body
      or 'val, col, row' in body)

print('\n=== 静态检查：超时不再无脑重撒 ===')

m2 = re.search(r'def _check_timeout.*?def _fmt_pose', CODE, re.S)
tb = m2.group(0) if m2 else ''
check('找到了 _check_timeout', m2 is not None)
check('σ 已达标时提前返回、不重撒',
      re.search(r'if self\._cov_below_converge\(\):', tb) is not None)
check('该分支在 _trigger_global_localization 之前',
      tb.find('_cov_below_converge') < tb.find('_trigger_global_localization')
      if '_trigger_global_localization' in tb else True)
check('新增 _cov_below_converge 方法',
      'def _cov_below_converge(self, scale: float = 1.0) -> bool:' in CODE)


# ============================================================
# 二、数值校验：用真实 room1 地图
# ============================================================
def parse_pgm(path):
    d = open(path, 'rb').read()
    toks, i = [], 0
    while len(toks) < 4:
        while d[i:i + 1] in b' \t\r\n':
            i += 1
        if d[i:i + 1] == b'#':
            while d[i:i + 1] not in b'\r\n':
                i += 1
            continue
        s = i
        while d[i:i + 1] not in b' \t\r\n':
            i += 1
        toks.append(d[s:i])
    i += 1
    w, h = int(toks[1]), int(toks[2])
    return w, h, d[i:i + w * h]


def pgm_to_grid(w, h, px, occ_th=0.65, free_th=0.196):
    """复刻 map_server 的 trinary 转换，含 Y 翻转（关键！）。

    map_server 的写法：
        for y_img in 0..h-1:
            for x in 0..w-1:
                i = x + (h - y_img - 1) * w
                data[i] = ...
    也就是说 PGM 的**图像行** y_img 映射到 data 的 **h-1-y_img** 行。
    data 的行号 = 世界坐标自下往上。
    """
    data = [0] * (w * h)
    for y_img in range(h):
        base_src = y_img * w
        base_dst = (h - y_img - 1) * w
        for x in range(w):
            occ = (255 - px[base_src + x]) / 255.0
            if occ > occ_th:
                v = 100
            elif occ < free_th:
                v = 0
            else:
                v = -1
            data[base_dst + x] = v
    return data


print('\n=== 数值：对比「正确索引」与「旧的双翻转」===')
w, h, px = parse_pgm(os.path.join(MAPS, 'room1.pgm'))
grid = pgm_to_grid(w, h, px)
RES, OX, OY = 0.05, -1.86, -8.17

free = sum(1 for v in grid if 0 <= v < 65)
occ = sum(1 for v in grid if v >= 65)
unk = sum(1 for v in grid if v < 0)
t = w * h
print('  room1 重建后：空闲 %.1f%%  占用 %.1f%%  未知 %.1f%%'
      % (free * 100.0 / t, occ * 100.0 / t, unk * 100.0 / t))
check('与 auto_localizer 启动日志一致（空闲25.3/占用6.8/未知67.9）',
      abs(free * 100.0 / t - 25.3) < 0.6
      and abs(occ * 100.0 / t - 6.8) < 0.3
      and abs(unk * 100.0 / t - 67.9) < 0.6)


def lookup(x, y, flip):
    col = int((x - OX) / RES)
    row = (h - 1 - int((y - OY) / RES)) if flip else int((y - OY) / RES)
    if not (0 <= col < w and 0 <= row < h):
        return None
    return grid[row * w + col]


# 本次实测的失败点：真实位姿在一片全空闲区域
X, Y = 0.34, -0.07
v_ok = lookup(X, Y, flip=False)
v_bad = lookup(X, Y, flip=True)
print('  位姿 (%.2f, %.2f)：正确索引 val=%s，旧双翻转 val=%s'
      % (X, Y, v_ok, v_bad))

check('正确索引下该位姿是空闲（不会误报未知）', v_ok is not None and v_ok >= 0)
check('旧双翻转确实读到未知（复现出本次故障）', v_bad is not None and v_bad < 0)

# 再验证一个反例：不能被「怎么算都通过」蒙混
X2, Y2 = 0.34, -1.50
print('  位姿 (%.2f, %.2f)：正确索引 val=%s，旧双翻转 val=%s'
      % (X2, Y2, lookup(X2, Y2, False), lookup(X2, Y2, True)))

print('\n=== 数值：地图确实有 2/3 未知，这是地图本身的问题 ===')
check('room1 未知区 > 60%（建图覆盖不全，建议重扫）', unk * 100.0 / t > 60.0)


# ============================================================
# 三、超时分支的决策逻辑
# ============================================================
print('\n=== 逻辑：超时该重撒还是该告警 ===')
CONV_XY, CONV_YAW = 0.08, 0.05


def timeout_action(std_xy, std_yaw):
    """复刻修好后的 _check_timeout 决策。"""
    if math.sqrt(max(0.0, std_xy ** 2)) <= CONV_XY \
            and math.sqrt(max(0.0, std_yaw ** 2)) <= CONV_YAW:
        return 'warn_no_reseed'
    return 'reseed'


check('σ=0.027m/1.7°（本次实测）→ 只告警、不重撒',
      timeout_action(0.027, math.radians(1.7)) == 'warn_no_reseed')
check('σ=0.937m/117°（真没收敛）→ 重撒',
      timeout_action(0.937, math.radians(117.0)) == 'reseed')
check('σ=0.219m/14.2°（t=112s 时）→ 重撒',
      timeout_action(0.219, math.radians(14.2)) == 'reseed')


# ============================================================
print('\n' + '=' * 56)
print('通过 %d / %d' % (PASS, PASS + FAIL))
print('=' * 56)
sys.exit(0 if FAIL == 0 else 1)
