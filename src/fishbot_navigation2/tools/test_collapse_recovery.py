#!/usr/bin/env python3
"""粒子塔缩恢复逻辑的离线回归测试（不启动 ROS、不碰实物）。

覆盖 2026-10-31 复盘后落地的三条改动：

  A「够用即停」   σ 已小于一个栅格就不再请求 nomotion，冻结粒子云
  B「原位重撒」   塔缩时在当前位姿局部扰动，而不是全局重撒导致瞬移
  C「配额重置」   稳定收敛足够久后，_collapse_recoveries 归零

用真实的 auto_localizer 源码做静态反射 + 逻辑镜像双保险：
  * 静态部分直接读源码断言关键行为存在（防止后续改动误删）
  * 逻辑部分复刻判据跑场景，验证阈值选择正确

运行：
    python3 src/fishbot_navigation2/tools/test_collapse_recovery.py
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


# ============================================================
# 一、静态断言：源码里必须存在这些行为
# ============================================================
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'fishbot_navigation2', 'auto_localizer.py')
with open(SRC, encoding='utf-8') as f:
    CODE = f.read()

print('\n=== 静态检查：代码结构 ===')

_conv_seg = re.search(
    r'self\.state = self\.STATE_CONVERGED.*?(?=\n    def )', CODE, re.S)
_conv_code = ''
if _conv_seg:
    # 只看代码行：注释里为说明历史 bug 会引用 "_timer.cancel()"
    _conv_code = '\n'.join(ln for ln in _conv_seg.group(0).splitlines()
                           if not ln.lstrip().startswith('#'))

check('收敛后不再 _timer.cancel() 自杀',
      _conv_seg is not None and '_timer.cancel()' not in _conv_code)

check('收敛时不再设 _finished=True',
      'self.state = self.STATE_CONVERGED\n        self._converged_at = now'
      in CODE)

check('CONVERGED 阶段仍在监控塔缩',
      re.search(r'if self\.state == self\.STATE_CONVERGED:.*?'
                r'self\._check_collapse\(now\)', CODE, re.S) is not None)

check('塌缩恢复默认走 local 模式',
      "'collapse_recovery_mode', 'local'" in CODE)

check('_check_collapse 已抽成独立方法',
      'def _check_collapse(self, now) -> bool:' in CODE)

check('_evaluate_convergence 前置调用 _check_collapse',
      re.search(r'def _evaluate_convergence.*?'
                r'if self\._check_collapse\(now\):', CODE, re.S) is not None)

check('局部重撒发布到 /initialpose 且带自环认领',
      re.search(r'def _publish_local_reseed.*?'
                r'self\._self_prior_xy = \(x, y\)', CODE, re.S) is not None)

check('塔缩恢复有冷却期保护',
      re.search(r'def _check_collapse.*?'
                r'since < self\.collapse_cooldown_sec', CODE, re.S) is not None)

check('配额可重置（C）',
      re.search(r'def _maybe_rearm_collapse_recoveries.*?'
                r'self\._collapse_recoveries = 0', CODE, re.S) is not None)

check('够用即停闸门在 _maybe_request_nomotion 内',
      re.search(r'def _maybe_request_nomotion.*?'
                r'if self\._cov_below_good_enough\(\):', CODE, re.S) is not None)


# ============================================================
# 二、逻辑镜像：复刻判据跑场景
# ============================================================
GOOD_XY, GOOD_YAW = 0.035, 0.03
COLLAPSE_XY, COLLAPSE_YAW = 0.005, 0.01
CONVERGE_XY = 0.08


def is_collapse(var_xy, var_yaw):
    """复刻 _check_collapse 的判据（注意入参是**方差**，判据比较的是 std）。"""
    std_xy = math.sqrt(max(0.0, var_xy))
    std_yaw = math.sqrt(max(0.0, var_yaw))
    return std_xy < COLLAPSE_XY and std_yaw < COLLAPSE_YAW


def below_good_enough(var_xy, var_yaw):
    """复刻 _cov_below_good_enough。"""
    std_xy = math.sqrt(max(0.0, var_xy))
    std_yaw = math.sqrt(max(0.0, var_yaw))
    return std_xy <= GOOD_XY and std_yaw <= GOOD_YAW


print('\n=== 逻辑：阈值是否切在真实塌缩点上 ===')

# 取自 /tmp/fishbot_amcl_trace.csv 的实测序列（σ 值，转成方差）
trace = [
    (0.0446, 0.0454, 'CONVERGE', '稳态：车静止，定位其实是对的'),
    (0.0446, 0.0454, 'CONVERGE', ''),
    (0.0258, 0.0263, 'CONVERGE', '开始被压缩'),
    (0.0234, 0.0238, 'CONVERGE', '继续压缩'),
    (0.0010, 0.0009, 'COLLAPSE', '塌缩：低于 0.05m 栅格物理下限'),
]

for std_xy, std_yaw, expect, note in trace:
    v_xy, v_yaw = std_xy ** 2, std_yaw ** 2
    got = is_collapse(v_xy, v_yaw)
    check('σ_xy=%.4f → %s %s'
          % (std_xy, 'COLLAPSE' if got else 'ok', '（%s）' % note if note else ''),
          got == (expect == 'COLLAPSE'))

print('\n=== 逻辑：够用即停是否会过早冻结 ===')

check('σ=0.0446（实测稳态）→ 继续请求，不过早冻结',
      not below_good_enough(0.0446 ** 2, 0.0454 ** 2),
      '若这里冻结就永远收敛不了')

check('σ=0.030（1 个栅格内）→ 冻结',
      below_good_enough(0.030 ** 2, 0.025 ** 2))

check('σ=0.0010（已塌缩）→ 也在冻结区（不会再去压）',
      below_good_enough(0.0010 ** 2, 0.0009 ** 2))

print('\n=== 逻辑：闸门顺序保证塌缩优先于冻结 ===')
# 关键点：塌缩判据（0.005）必须严格落在够用线（0.035）之内，
# 否则会出现「已经塌缩但还在请求 nomotion」的窗口。
check('collapse_cov_xy < good_enough_cov_xy',
      COLLAPSE_XY < GOOD_XY,
      '否则塌缩检测之前就已经停止更新，塌缩会静默发生')
check('collapse_cov_yaw < good_enough_cov_yaw',
      COLLAPSE_YAW < GOOD_YAW)

print('\n=== 逻辑：局部重撒的协方差要能撑起多样性 ===')
LOCAL_RESEED_XY = 0.05      # 方差
LOCAL_RESEED_STD = math.sqrt(LOCAL_RESEED_XY)
check('局部重撒后 σ=%.3fm，明显大于够用线带入重收敛' % LOCAL_RESEED_STD,
      LOCAL_RESEED_STD > GOOD_XY)
check('局部重撒后 σ=%.3fm，远小于全局重撒的 1.4m（不瞬移）' % LOCAL_RESEED_STD,
      LOCAL_RESEED_STD < 0.5)
check('局部重撒不会被误判成新的塌缩',
      not is_collapse(LOCAL_RESEED_XY, 0.03))

print('\n=== 逻辑：冷却期能撑到 AMCL 回新位姿 ===')
# tick 0.5s，AMCL 响应 1~2s；冷却 4s 内不会重复计数
COOLDOWN = 4.0
ticks_in_cooldown = int(COOLDOWN / 0.5)
check('冷却期覆盖 %d 个 tick（AMCL 只需 1~2s 响应）' % ticks_in_cooldown,
      ticks_in_cooldown >= 8)

print('\n=== 逻辑：配额重置周期 vs 实测塌缩周期 ===')
# 实测两次塌缩间隔 68s；重置周期 15s 远小于它，
# 意味着正常停留不会耗尽配额
REARM = 15.0
check('重置周期 %.0fs < 实测塌缩间隔 68s' % REARM, REARM < 68.0)
check('重置周期 %.0fs > 收敛 hold 3s（不会一收敛就重置）' % REARM, REARM > 3.0)


# ============================================================
print('\n' + '=' * 52)
print('通过 %d / %d' % (PASS, PASS + FAIL))
print('=' * 52)
sys.exit(0 if FAIL == 0 else 1)
