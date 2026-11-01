#!/usr/bin/env python3
"""
test_loc_gate.py
============================================================
「定位未收敛不许导航」闸门的回归测试（2026-11-01 实机事故）

事故现场（auto_localizer + amcl_tracer 日志）：
    11:47:25  载入地图 → 全局定位开始（粒子云撒满全图）
    11:47:25 ~ 11:50:03   σ_xy 一直在 0.11 ~ 1.69m 之间，**从未收敛**
    11:48:05  却启动了 return_to_start       → 车按错误位姿开动
    11:49:49  又启动了去厕所的导航            → σ_xy 被拉到 1.69m
    11:49:25  120s 超时 → 重新撒粒子           → 位姿全图乱跳
    用户看到：「小车在地图上的位姿有规律乱飘」

本测试不启动 ROS、不接触硬件，纯静态 + 逻辑镜像。
"""

import os
import re
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', 'autopatrol_robot')
NAV_MGR = os.path.join(PKG, 'nav_manager.py')
NAV_SVC = os.path.join(PKG, 'navigation_service.py')
PATROL = os.path.join(PKG, 'patrol_node.py')
NAV_TO_POSE = os.path.join(HERE, '..', '..', 'fishbot_application',
                           'fishbot_application', 'nav_to_pose.py')

ok = 0
total = 0


def check(name, cond, extra=''):
    global ok, total
    total += 1
    ok += bool(cond)
    print('  %s %s %s' % ('✅' if cond else '❌', name, extra))


def read(path):
    with open(path, encoding='utf-8') as f:
        return f.read()


# ============================================================
# 一、静态检查：四处导航入口都要装闸门
# ============================================================
print('一、导航入口的定位闸门')

nav_code = read(NAV_MGR)

check('nav_manager 定义了 _localization_block_msg',
      'def _localization_block_msg(' in nav_code)

launch_funcs = ['launch_navigate_by_index', 'launch_navigate_sequence',
                'launch_return_to_start', 'launch_navigate_xy']
for fn in launch_funcs:
    m = re.search(r'def %s\(.*?\n(?=^def |\Z)' % fn, nav_code,
                  re.S | re.M)
    body = m.group(0) if m else ''
    check('%s 调用了定位闸门' % fn, '_localization_block_msg()' in body)
    check('%s 闸门在 _preempt_active_nav 之前' % fn,
          body.find('_localization_block_msg()')
          < body.find('_preempt_active_nav()'))

check('可用 FISHBOT_NAV_IGNORE_LOC=1 关闭闸门',
      'FISHBOT_NAV_IGNORE_LOC' in nav_code)
check('阈值可通过环境变量调整',
      'FISHBOT_LOC_OK_STD_XY' in nav_code
      and 'FISHBOT_LOC_OK_STD_YAW' in nav_code)

# ============================================================
# 二、静态检查：navigation_service 暴露定位质量
# ============================================================
print()
print('二、定位质量接口')

svc_code = read(NAV_SVC)

check('缓存了协方差 _pose_std', '_pose_std' in svc_code)
check('定义了 get_localization_quality',
      'def get_localization_quality(self)' in svc_code)
check('协方差索引用 [0]/[7]/[35]（x/y/yaw）',
      'cov[0]' in svc_code and 'cov[7]' in svc_code
      and 'cov[35]' in svc_code)
check('从未收到 /amcl_pose 时放行（不误挡）',
      '尚未收到 /amcl_pose，无法判断定位质量' in svc_code)
check('位姿过期时放行（AMCL 重启中不误挡）',
      'AMCL 位姿已过期' in svc_code)


# ============================================================
# 三、逻辑镜像：闸门判定
# ============================================================
print()
print('三、闸门判定逻辑')


def gate(std_xy, std_yaw, age=0.5, ok_xy=0.30, ok_yaw=0.30):
    """镜像 get_localization_quality 的判定。"""
    if std_xy is None or std_yaw is None:
        return True
    if age > 3.0:
        return True
    return std_xy <= ok_xy and std_yaw <= ok_yaw


cases = [
    # (σ_xy, σ_yaw, 期望放行, 说明)
    (0.05, 0.03, True, '定位良好'),
    (0.30, 0.30, True, '恰好等于门限'),
    (0.31, 0.05, False, '平移略超门限'),
    (0.05, 0.35, False, '朝向超门限'),
    (0.11, 0.20, True, '事故里 72s 时的最好状态（应放行）'),
    (1.69, 0.55, False, '事故里 113s 时的状态（应拦截）'),
    (None, None, True, '无数据（放行，避免冷启动全锁死）'),
]
for std_xy, std_yaw, want, why in cases:
    got = gate(std_xy, std_yaw)
    check('%-32s σ=%-5s/%-5s → %s' % (why, std_xy, std_yaw,
                                      '放行' if got else '拦截'),
          got == want)

check('过期位姿放行',
      gate(9.9, 9.9, age=5.0) is True)

# ============================================================
# 四、僵尸防护
# ============================================================
print()
print('四、导航循环僵尸防护')

ntp = read(NAV_TO_POSE)
check('nav_to_pose 有失联超时常量', 'STALL_TIMEOUT_SEC' in ntp)
check('nav_to_pose 有墙钟超时常量', 'MAX_WALL_SEC' in ntp)
check('nav_to_pose 检测反馈不变化',
      'last_eta_change' in ntp)
check('nav_to_pose 失联时 break 退出循环',
      re.search(r'STALL_TIMEOUT_SEC:\s*\n.*?break', ntp, re.S) is not None)
check('nav_to_pose 处理 getFeedback() 为 None',
      'if feedback is not None' in ntp)

patrol = read(PATROL)
check('patrol_node 有失联超时常量', 'NAV_STALL_TIMEOUT_SEC' in patrol)
check('patrol_node 有墙钟超时常量', 'NAV_MAX_WALL_SEC' in patrol)
check('patrol_node 两处导航循环都装了防护',
      patrol.count('last_fb_change') >= 4,
      '出现 %d 次' % patrol.count('last_fb_change'))

# ============================================================
# 五、重复播报
# ============================================================
print()
print('五、NavPhase 重复播报')

check('_monitor_process 有 report 开关',
      'def _monitor_process(report: bool = True)' in check.__globals__['read'](
          NAV_MGR))
check('带返航的流程把 monitor 静音',
      nav_code.count('silent=True') >= 3,
      '出现 %d 次' % nav_code.count('silent=True'))
# ★ 2026-11-01：『导航后自动返回起点』流程已彻底删除，
#   所以 worker 里不再有 launch_return_to_start 调用。
check('worker 里不再自动返航（流程已删除）',
      'launch_return_to_start(silent=True)' not in nav_code)

# ============================================================
# 六、启动清理
# ============================================================
print()
print('六、僵尸进程清理')

web_code = read(os.path.join(PKG, 'web_controller.py'))
check('清理特征包含 nav_to_pose',
      'fishbot_application/lib/fishbot_application/nav_to_pose' in web_code)
check('清理特征包含 patrol_node',
      'autopatrol_robot/lib/autopatrol_robot/patrol_node' in web_code)
check('web_controller 启动时清理上一轮遗留进程',
      '启动清理' in web_code and '_reap_orphan_nav_processes' in web_code)

# ============================================================
# 七、auto_localizer 不再反复重撒
# ============================================================
print()
print('七、auto_localizer 重撒策略')

al = read(os.path.join(HERE, '..', '..', 'fishbot_navigation2',
                       'fishbot_navigation2', 'auto_localizer.py'))
check('新增 almost_converge_scale 参数',
      "declare_parameter('almost_converge_scale'" in al)
check('_cov_below_converge 支持放宽倍数',
      'def _cov_below_converge(self, scale: float = 1.0)' in al)
check('接近收敛时不重撒',
      al.find('almost_converge_scale)') <
      al.find('重新撒粒子重试'))
check('不重撒时延长截止时间',
      '_nearly_converged_reported' in al)

print()
print('=' * 60)
print('通过 %d / %d' % (ok, total))
print('=' * 60)
sys.exit(0 if ok == total else 1)
