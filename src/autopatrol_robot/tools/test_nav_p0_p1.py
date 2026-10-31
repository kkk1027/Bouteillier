#!/usr/bin/env python3
"""P0（TF 陈旧）/ P1（孤儿进程）修复的离线回归测试。

不启动 ROS、不碰实物。

P0 背景（2026-10-31 21:45 回起点实测）：
    Exception in transformPose: Lookup would require extrapolation into
    the future. Requested 1793454305.734 but latest data is 1793454304.859785
    → Controller patience exceeded → [follow_path] Aborting handle
    一轮导航出现 5 次，map→odom 滞后 0.87s。
    根因：stop_nomotion_after_converge=True 让 AMCL 收敛后只在激光到达
    时才发 TF，而雷达经 WiFi、有效帧率仅 0.86Hz。

P1 背景：
    nav_to_pose 孤儿进程存活 3 小时 10 分。web_controller 重启后
    _active_nav_process 重置为 None，daemon 监控线程消失，子进程
    因 preexec_fn=os.setsid 独立成会话组，再无人回收。

运行：
    python3 src/autopatrol_robot/tools/test_nav_p0_p1.py
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
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
NAV2_YAML = os.path.join(WS, 'src', 'fishbot_navigation2', 'config',
                         'nav2_params.yaml')
LOCALIZER = os.path.join(WS, 'src', 'fishbot_navigation2',
                         'fishbot_navigation2', 'auto_localizer.py')
NAV_MGR = os.path.join(WS, 'src', 'autopatrol_robot', 'autopatrol_robot',
                       'nav_manager.py')

# ============================================================
# P0-1：transform_tolerance 必须够大
# ============================================================
print('\n=== P0-1：transform_tolerance ===')

try:
    import yaml
    _amcl = yaml.safe_load(open(NAV2_YAML, encoding='utf-8'))[
        'amcl']['ros__parameters']
    tt = float(_amcl['transform_tolerance'])
    print('  实测值：%.2f s' % tt)
    check('transform_tolerance ≥ 1.0（nav2 默认值）', tt >= 1.0,
          '实测 map→odom 滞后 0.87s，0.2 撑不住')
    check('不再是历史值 0.2', abs(tt - 0.2) > 1e-6)
except Exception as exc:                               # noqa: BLE001
    check('读取 nav2_params.yaml', False, str(exc))

# 其余防塌缩参数不能被回退
try:
    check('update_min_d 仍为 0.15（防静止抖动触发重采样）',
          abs(float(_amcl['update_min_d']) - 0.15) < 1e-6)
    check('resample_interval 仍为 2（重采样频率减半）',
          int(_amcl['resample_interval']) == 2)
    check('recovery_alpha_slow 仍为 0.002（随机粒子不激进）',
          abs(float(_amcl['recovery_alpha_slow']) - 0.002) < 1e-9)
except Exception as exc:                               # noqa: BLE001
    check('读取防塌缩参数', False, str(exc))

# ============================================================
# P0-2：收敛后不能再"停发 nomotion"
# ============================================================
print('\n=== P0-2：收敛后仍刷新 map→odom ===')

CODE = open(LOCALIZER, encoding='utf-8').read()

m = re.search(r'def _maybe_request_nomotion.*?(?=\n    def )', CODE, re.S)
body = m.group(0) if m else ''
code_only = '\n'.join(ln for ln in body.splitlines()
                      if not ln.lstrip().startswith('#'))

check('找到了 _maybe_request_nomotion', m is not None)
check('不再按状态直接 return（收敛→停发的旧分支已删）',
      'if self.state == self.STATE_CONVERGED:' not in code_only)
check('不再读取 self.stop_nomotion_after_converge',
      'self.stop_nomotion_after_converge' not in code_only)
check('改为统一 σ 闸门 _cov_below_good_enough',
      'self._cov_below_good_enough()' in code_only)
check('σ 涨回来会恢复请求（_nomotion_suspended 复位）',
      '_nomotion_suspended = False' in code_only)

# 参数仍声明（向后兼容旧 launch 传参），但默认 False
check('stop_nomotion_after_converge 仍声明以便旧 launch 不报错',
      "declare_parameter('stop_nomotion_after_converge'" in CODE)
check('其默认值已改为 False',
      re.search(r"declare_parameter\('stop_nomotion_after_converge',\s*"
                r"False\)", CODE) is not None)

# ============================================================
# P0-3：闸门逻辑自洽（塌缩判据必须比够用线更严格）
# ============================================================
print('\n=== P0-3：闸门阈值自洽 ===')


def parse_param(name, default_type=float):
    mm = re.search(r"declare_parameter\('%s',\s*([-\d.]+)\)" % name, CODE)
    return default_type(mm.group(1)) if mm else None


good_xy = parse_param('good_enough_cov_xy')
good_yaw = parse_param('good_enough_cov_yaw')
col_xy = parse_param('collapse_cov_xy')
col_yaw = parse_param('collapse_cov_yaw')
conv_xy = parse_param('converge_cov_xy')
conv_yaw = parse_param('converge_cov_yaw')

print('  collapse=(%.4f, %.4f)  够用=(%.3f, %.3f)  收敛=(%.2f, %.2f)'
      % (col_xy, col_yaw, good_xy, good_yaw, conv_xy, conv_yaw))

check('塌缩线 < 够用线（塌缩优先于冻结）',
      col_xy < good_xy and col_yaw < good_yaw)
check('够用线 < 收敛线（够用即停不会阻止收敛判定）',
      good_xy < conv_xy and good_yaw < conv_yaw)
check('够用线 0.035m ≈ 半个栅格（0.05m 地图）', 0.02 <= good_xy <= 0.06)


def nomotion_state(var_xy, var_yaw):
    """复刻修好后的 _maybe_request_nomotion 决策。"""
    std_xy = math.sqrt(max(0.0, var_xy))
    std_yaw = math.sqrt(max(0.0, var_yaw))
    if std_xy <= good_xy and std_yaw <= good_yaw:
        return 'suspended'
    return 'request'


check('σ=0.29m（本次实测 DIVERGE 值）→ 继续请求（保住 TF 新鲜）',
      nomotion_state(0.29 ** 2, math.radians(5.0) ** 2) == 'request')
check('σ=0.074m（收敛瞬间）→ 继续请求',
      nomotion_state(0.074 ** 2, math.radians(2.0) ** 2) == 'request')
check('σ=0.030m（静止久了）→ 暂停（防塌缩）',
      nomotion_state(0.030 ** 2, 0.02 ** 2) == 'suspended')

# ============================================================
# P1：孤儿进程清理
# ============================================================
print('\n=== P1：孤儿进程清理 ===')

NM = open(NAV_MGR, encoding='utf-8').read()

check('新增 _reap_orphan_nav_processes 函数',
      'def _reap_orphan_nav_processes()' in NM)
check('匹配 nav_to_pose', "'ros2 run fishbot_application nav_to_pose'" in NM)
check('匹配 autopatrol.launch.py',
      "'ros2 launch autopatrol_robot autopatrol.launch.py'" in NM)

n_calls = len(re.findall(r'^\s+_reap_orphan_nav_processes\(\)$', NM, re.M))
n_popen = len(re.findall(r'_active_nav_process = subprocess\.Popen\(', NM))
print('  Popen 启动点 %d 个，清场调用 %d 处' % (n_popen, n_calls))
check('每个导航启动点前都清场', n_calls == n_popen and n_popen >= 4)

check('清场用 killpg 杀整个会话组（子进程 setsid 过）',
      'os.killpg(os.getpgid(pid), signal.SIGKILL)' in NM)
check('不会误杀自己', 'if pid == me or pid in keep:' in NM)
check('不会误杀仍活跃的当前导航',
      re.search(r'_active_nav_process is not None and '
                r'_active_nav_process\.poll\(\) is None', NM) is not None)

# ============================================================
print('\n' + '=' * 56)
print('通过 %d / %d' % (PASS, PASS + FAIL))
print('=' * 56)
sys.exit(0 if FAIL == 0 else 1)
