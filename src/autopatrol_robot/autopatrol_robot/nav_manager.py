"""
nav_manager.py
====================================================
CLI 导航进程生命周期管理器。

AI 工具通过 NavManager 启动/取消/查询导航 —— 全部走 CLI 子进程，
与 Web UI 使用的 NavigationService 完全解耦。

全局互斥锁 _global_nav_lock 与 NavigationService 共享，
防止 Web UI 和 AI 同时发起导航。

═══════════════════════════════════════════════════════════════
⚠️  关键设计决策：stdout 直接丢弃（DEVNULL）
═══════════════════════════════════════════════════════════════
所有 CLI 子进程使用 stdout=subprocess.DEVNULL，避免管道缓冲区
（OS 默认 64KB）在 patrol_node 高频日志下填满导致 write() 阻塞死锁。
代价：导航日志不保留，排查需看 ros2 日志文件 (~/.ros/log/)。
"""

import os
import signal
import subprocess
import threading
import time
from typing import Optional

# ============================================================
# 全局互斥锁 —— Web UI 和 AI 导航共享
# ============================================================
_global_nav_lock = threading.Lock()

# AI 侧 CLI 进程管理
_active_nav_process: Optional[subprocess.Popen] = None
_active_nav_target: str = ""

# 导航状态变更回调（供 Web UI 推送到达通知）
_nav_callbacks: list = []  # 每个元素: callable(phase: str, message: str)


def register_nav_callback(cb):
    """注册导航阶段变更回调。phase: 'started'|'arrived'|'returning'|'completed'|'failed'"""
    _nav_callbacks.append(cb)


def unregister_nav_callback(cb):
    """取消注册回调"""
    try:
        _nav_callbacks.remove(cb)
    except ValueError:
        pass


def _notify_callbacks(phase: str, message: str):
    """通知所有已注册的回调"""
    for cb in _nav_callbacks:
        try:
            cb(phase, message)
        except Exception:
            pass


def _get_fishbot_ws() -> str:
    # 原先默认值是原作者机器的 '/home/ke/chapt9_update/fishbot_ws'，
    # 换机器后所有 ros2 launch 都会失败。统一由 paths.py 定位。
    from autopatrol_robot.paths import fishbot_ws
    return fishbot_ws()


def _cleanup_process(proc: Optional[subprocess.Popen]):
    """安全回收子进程：wait 防僵尸。"""
    if proc is None:
        return
    try:
        proc.wait(timeout=1)
    except Exception:
        pass


# ============================================================
# 孤儿导航进程清理
# ------------------------------------------------------------
# 问题（2026-10-31 实测）：nav_to_pose 进程存活了 3 小时 10 分，
# 而当时的导航栈 3 小时前就已经换了新的。
#
# 成因：
#   1. Popen 用了 preexec_fn=os.setsid，子进程独立成会话组，
#      不受父进程（web_controller）的进程组信号影响；
#   2. 回收依赖 daemon=True 的 _monitor_process 线程 poll()；
#   3. web_controller 一重启，_active_nav_process 被重置成 None，
#      监控线程随旧进程消失 —— 子进程彻底失去管控，永远留着。
#
# 后果：孤儿仍占着 /navigate_to_pose 的 action client，可能和新导航
# 互相打断，或在下一次导航时突然发出自己那个过期的目标。
#
# 对策：每次启动新导航前主动扫一遍，把不属于当前活跃进程的全清掉。
# 全局锁已保证「同一时刻只有一个导航」，所以此刻扫到的都是孤儿。
# ============================================================
_NAV_PROC_MARKERS = (
    'ros2 run fishbot_application nav_to_pose',
    'ros2 launch autopatrol_robot autopatrol.launch.py',
)


def _reap_orphan_nav_processes() -> int:
    """杀掉所有遗留的导航子进程，返回清理个数。

    必须在持有 _global_nav_lock 的情况下调用。
    """
    keep = set()
    if _active_nav_process is not None and _active_nav_process.poll() is None:
        keep.add(_active_nav_process.pid)
        keep.add(os.getpgid(_active_nav_process.pid))

    me = os.getpid()
    killed = 0
    for marker in _NAV_PROC_MARKERS:
        try:
            out = subprocess.run(
                ['pgrep', '-f', marker],
                capture_output=True, text=True, timeout=3,
            ).stdout
        except Exception:
            continue
        for line in out.split():
            if not line.isdigit():
                continue
            pid = int(line)
            # 别杀自己、别杀还活着的当前导航
            if pid == me or pid in keep:
                continue
            try:
                if os.getpgid(pid) in keep:
                    continue
                os.killpg(os.getpgid(pid), signal.SIGKILL)
                killed += 1
                print('[导航] 已清理遗留导航进程 pid=%d (%s)'
                      % (pid, marker))
            except ProcessLookupError:
                pass
            except Exception as exc:                   # noqa: BLE001
                print('[导航] 清理 pid=%d 失败: %s' % (pid, exc))
    return killed


# ============================================================
# 与雷达跟随互斥
# ------------------------------------------------------------
# 本项目没有 twist_mux，导航和雷达跟随都直接往 /cmd_vel 发指令。
# 两个一起跑会互相覆盖：跟随想保持 50cm，导航想奔目标点，
# 结果是车在原地抽搐。
#
# 约定：跟随时写 /tmp/fishbot_lidar_follow.lock，
# 导航启动前看到这个锁就拒绝，并提示用户先关跟随。
# ============================================================
_LIDAR_FOLLOW_LOCK = "/tmp/fishbot_lidar_follow.lock"


def lidar_follow_active() -> bool:
    """雷达跟随是否正在占用底盘。

    以锁文件为准（跟随节点启动时写、退出时删）。
    同时验证进程还在，防止节点被 kill -9 后锁文件残留导致
    导航永远无法启动。

    ★ 检查进程时必须排除自身 PID。
    `pgrep -f "lidar_follow"` 会匹配命令行里含该字符串的任何进程，
    包括本进程自己 —— 本文件里的函数名就叫 lidar_follow_active，
    调用链一旦出现在命令行里，pgrep 就会把自己算进去，
    陈旧锁永远清不掉。本项目实际踩过这个坑。
    """
    if not os.path.exists(_LIDAR_FOLLOW_LOCK):
        return False
    try:
        out = subprocess.run(
            ["pgrep", "-f", "lidar_follow"],
            capture_output=True, text=True, timeout=3)
        me = os.getpid()
        alive = False
        for tok in out.stdout.split():
            try:
                pid = int(tok)
            except ValueError:
                continue
            if pid == me:
                continue
            alive = True
            break
        if alive:
            return True
        # 锁在但进程没了 → 陈旧锁，清掉
        try:
            os.remove(_LIDAR_FOLLOW_LOCK)
        except Exception:
            pass
        print("[导航] 检测到陈旧的跟随锁，已清理")
        return False
    except Exception:
        # pgrep 不可用时就相信锁文件，宁可拒一次也不要撞车
        return True


def _blocked_by_follow_msg() -> Optional[str]:
    if lidar_follow_active():
        return ("⚠️ 雷达跟随正在运行，无法启动导航（两者都会控制 /cmd_vel）。\n"
                "请先说「停止雷达跟随」，或在网页上关闭跟随，然后再导航。")
    return None


# ============================================================
# 定位质量闸门（2026-11-01 实机事故）
# ------------------------------------------------------------
# 现场日志（auto_localizer + amcl_tracer）：
#   11:47:25  载入地图 → 全局定位开始（粒子云撒满全图）
#   11:47:25 ~ 11:50:03   σ_xy 一直在 0.11 ~ 1.69m 之间晃，**从未收敛**
#   11:48:05  却启动了 return_to_start  → 车按错误位姿开动
#   11:49:49  又启动了去厕所的导航        → σ_xy 被拉到 1.69m
#   11:49:25  120s 超时 → 重新撒粒子     → 位姿再次全图乱跳
#   结果：用户看到「小车在地图上的位姿有规律乱飘」
#
# 责任划分很清楚：**定位没收敛时就不该允许导航**。
# 车按错误位姿行驶只会让 σ 越来越大，进而触发重撒，形成正反馈。
#
# 阈值可通过环境变量调整：
#   FISHBOT_LOC_OK_STD_XY   默认 0.30 m
#   FISHBOT_LOC_OK_STD_YAW  默认 0.30 rad
#   FISHBOT_NAV_IGNORE_LOC=1  彻底关闭本闸门（调试用）
# ============================================================
def _localization_block_msg() -> Optional[str]:
    """定位未收敛时返回拒绝原因，否则 None。"""
    if os.environ.get('FISHBOT_NAV_IGNORE_LOC', '') == '1':
        return None
    try:
        from autopatrol_robot.navigation_service import (
            get_global_nav_service)
        svc = get_global_nav_service()
        if svc is None:
            return None            # 导航服务还没初始化，无从判断，放行
        ok, std_xy, std_yaw, why = svc.get_localization_quality()
    except Exception as exc:       # noqa: BLE001 - 判断失败不能挡住导航
        print('[导航] ⚠️ 定位质量检查失败（不影响导航）: %s' % exc)
        return None

    if ok:
        return None

    print('[导航] 🚫 定位未就绪，拒绝启动导航：%s' % why)
    return (
        "❌ 定位还没准备好，现在出发会走错方向。\n"
        "   %s\n"
        "   全局定位需要**移动/转动小车**才能收敛（静止时几乎不动）。\n"
        "   请先缓慢推动或原地转动小车几秒，等网页上的定位状态条变绿再下指令。"
        % why.replace('\n', '\n   ')
    )


# ============================================================
# 导航抢占（preempt）：新目标自动中断旧目标
# ------------------------------------------------------------
# 【为什么必须抢占 —— 2026-11-01 实机日志实锤】
#
# 现场对话（agent_node 原始日志）：
#   [10] 用户: 回起点
#   [11] AI  : tool_calls=[return_to_start]
#   [13] AI  : 已返回起点，请主人稍等片刻。
#   [14] 用户: 去厕所                       ← 车其实还在返航途中
#   [15] AI  : tool_calls=[navigate_to_saved_location]
#   [16] Tool: ❌ 启动导航失败：⚠️ 已有导航任务进行中
#              （目标：回起点），请先取消
#   [17] AI  : 请先取消当前的导航任务，我才能回起点。   ← 复读了错误消息
#   [18] 用户: 去厕所
#   [19] AI  : tool_calls=[return_to_start]   ← ★ 又回起点了，任务彻底跑偏
#
# 三个连锁问题：
#   1. 用户体验荒谬 —— 说「去厕所」却被告知「请先取消回起点」；
#   2. 错误消息里**带着旧目标名**（"回起点"），3B 模型直接把它当成了
#      用户意图，下一轮就真的调了 return_to_start；
#   3. 3B 模型不会自己组成「先 cancel_navigation 再 navigate」两步，
#      它只会复读错误文本。
#
# 结论：在车里说一个新目的地 == 改主意，旧任务应当立即作废。
# 这个判断放在**代码层**做，而不是指望模型去理解错误消息。
# ============================================================

# 被主动打断的导航进程 pid（供后台 worker 区分「失败」和「被抢占」）
#
# ★ 用 pid 而不是目标名：本轮可能存在同名任务（去 A 回起点，
#   途中又被叫一次回起点），按名字匹配会把新生效的任务误当成
#   已被抢占的那个，真实的失败就会被静默吞掉。
_preempted_pids: set = set()


def _preempt_active_nav() -> str:
    """中断当前导航，返回被中断的目标名（没有则返回空串）。

    ★ 调用者必须已持有 _global_nav_lock。
    ★ 这里持锁 sleep 最多 2s 是刻意的：一是不让并发 launch 插队，
      二是 _monitor_process 只在极短临界区里取锁，不会被明显拖慢。
    """
    global _active_nav_process, _active_nav_target

    proc = _active_nav_process
    if proc is None or proc.poll() is not None:
        _active_nav_process = None
        _active_nav_target = ""
        return ""

    target = _active_nav_target or "上一个任务"
    old_pid = proc.pid
    try:
        # SIGINT 让 nav2 走正常取消流程（会发 /cmd_vel 零速），
        # 直接 SIGKILL 可能把底盘留在上一次的速度指令上。
        os.killpg(os.getpgid(old_pid), signal.SIGINT)
    except Exception:
        pass

    deadline = time.time() + 2.0
    while time.time() < deadline and proc.poll() is None:
        time.sleep(0.05)

    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
        try:
            proc.wait(timeout=1)
        except Exception:
            pass

    _active_nav_process = None
    _active_nav_target = ""
    _preempted_pids.add(old_pid)
    _cleanup_process(proc)
    print("[导航] 🛑 已中断上一个导航任务（%s），改为执行新目标" % target)
    return target


def _monitor_process(report: bool = True):
    """后台监控线程：等待 CLI 进程结束并清理状态。结束时会通知回调。

    report=False 时不发「completed / failed」回调 —— 供
    「导航 + 自动返航」这类由 _async_*_worker 接管上报的流程使用。

    ============================================================
    为什么要这个开关（2026-11-01 实机日志）
    ============================================================
    日志里同一次到达被播报了两遍：
        [NavPhase] completed: ✅ 导航完成：厕所
        [NavPhase] arrived:   ✅ 已到达 厕所！
    reason：launch_navigate_by_index 启动了 _monitor_process 盯这个
    subprocess，而 launch_navigate_with_return_async 又启动了
    后台 worker 盯同一个 subprocess。
    进程一退出，两个线程都拿到 rc==0，于是各自 notify 一次。

    而且语义也不对：对「到达后自动回起点」这个流程，真正的
    completed **应该是「已回到起点」**，而不是「到达目标」。
    所以带返航的流程一律静音 monitor，只由 worker 上报。
    """
    global _active_nav_process, _active_nav_target
    proc = _active_nav_process
    target = _active_nav_target
    if proc is None:
        return
    try:
        proc.wait(timeout=3600)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass

    rc = proc.returncode if proc.returncode is not None else -9
    with _global_nav_lock:
        if _active_nav_process is proc:
            _active_nav_process = None
            _active_nav_target = ""
    _cleanup_process(proc)

    if not report:
        return

    # 通知回调
    # ★ 被抢占打断（用户改主意）不算失败 —— 否则 UI 会先弹
    #   「❌ 导航失败：回起点」，再弹「📍 正在前往厕所」，很吓人。
    if rc != 0 and proc.pid in _preempted_pids:
        _preempted_pids.discard(proc.pid)
        print("[导航] 任务「%s」已被新指令抢占，跳过失败通知" % target)
        return
    if rc == 0:
        _notify_callbacks("completed", f"✅ 导航完成：{target}")
    else:
        _notify_callbacks("failed", f"❌ 导航失败：{target}（退出码 {rc}）")


# ============================================================
# Public API
# ============================================================

def launch_navigate_by_index(index: int, target_name: str = "",
                             silent: bool = False) -> str:
    """
    通过航点索引启动 CLI 导航（非阻塞，1-based）。

    Args:
        index: 航点索引（1-based，与 autopatrol.launch.py 的 target_indices 一致）
        target_name: 目标名称（用于日志和状态显示）
        silent: True 时不发回调（由上层 worker 统一上报，见 _monitor_process）

    Returns:
        结果描述字符串
    """
    global _active_nav_process, _active_nav_target

    with _global_nav_lock:
        _blk = _blocked_by_follow_msg()
        if _blk:
            return _blk
        loc_blk = _localization_block_msg()
        if loc_blk:
            return loc_blk
        # 说新目的地 = 改主意，旧任务立即作废（详见 _preempt_active_nav）
        preempted = _preempt_active_nav()

        fishbot_ws = _get_fishbot_ws()
        cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {fishbot_ws}/install/setup.bash && "
            f"ros2 launch autopatrol_robot autopatrol.launch.py "
            f"target_indices:={index}"
        )

        _reap_orphan_nav_processes()
        _active_nav_process = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        _active_nav_target = target_name or f"点{index}"
        new_target = _active_nav_target

    if preempted:
        _notify_callbacks(
            "preempted",
            f"🛑 已中断「{preempted}」，改为前往 {new_target}")

    # 启动监控线程
    threading.Thread(target=_monitor_process, args=(not silent,),
                     daemon=True).start()

    return f"📍 正在前往 {new_target}..."


def launch_navigate_sequence(indices: list, target_names: str = "",
                             silent: bool = False) -> str:
    """
    一次 subprocess 完成多目标序列导航（patrol_node 内部按序执行）。

    与逐个调用 launch_navigate_by_index 的区别：
      - 只启动 1 个 subprocess（而非 N 个）
      - 只执行 1 次 waitUntilNav2Active / rclpy.init / rclpy.shutdown
      - patrol_node 内部 for 循环依次导航每个目标，每个目标到达后
        执行舵机播报并等待按钮确认，再继续下一个。

    Args:
        indices: 航点索引列表（1-based），如 [2, 3, 1] 表示客厅→影音房→起点
        target_names: 用于日志描述的名称

    Returns:
        结果描述字符串
    """
    global _active_nav_process, _active_nav_target

    with _global_nav_lock:
        _blk = _blocked_by_follow_msg()
        if _blk:
            return _blk
        loc_blk = _localization_block_msg()
        if loc_blk:
            return loc_blk
        preempted = _preempt_active_nav()

        indices_str = ','.join(str(i) for i in indices)
        fishbot_ws = _get_fishbot_ws()
        cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {fishbot_ws}/install/setup.bash && "
            f"ros2 launch autopatrol_robot autopatrol.launch.py "
            f"target_indices:={indices_str}"
        )

        _reap_orphan_nav_processes()
        _active_nav_process = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        _active_nav_target = target_names or f"序列{indices_str}"
        new_target = _active_nav_target

    if preempted:
        _notify_callbacks(
            "preempted",
            f"🛑 已中断「{preempted}」，改为执行序列 {new_target}")

    threading.Thread(target=_monitor_process, args=(not silent,),
                     daemon=True).start()
    return f"📍 序列导航开始：{new_target}"


def launch_return_to_start(silent: bool = False) -> str:
    """
    启动返航到起点 —— 使用 fishbot_application 的 nav_to_pose 节点。

    与 launch_navigate_by_index 一致：通过 CLI 子进程运行，
    受全局互斥锁保护，与 Web UI 导航互斥。

    silent=True 时不发回调（供「导航+自动返航」worker 统一上报）。
    """
    global _active_nav_process, _active_nav_target

    with _global_nav_lock:
        _blk = _blocked_by_follow_msg()
        if _blk:
            return _blk
        loc_blk = _localization_block_msg()
        if loc_blk:
            return loc_blk
        preempted = _preempt_active_nav()

        fishbot_ws = _get_fishbot_ws()
        cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {fishbot_ws}/install/setup.bash && "
            f"ros2 run fishbot_application nav_to_pose"
        )

        _reap_orphan_nav_processes()
        _active_nav_process = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        _active_nav_target = "回起点"

    if preempted:
        _notify_callbacks(
            "preempted",
            f"🛑 已中断「{preempted}」，改为返回起点")

    threading.Thread(target=_monitor_process, args=(not silent,),
                     daemon=True).start()
    return "📍 正在返回起点..."


def launch_navigate_xy(x: float, y: float, yaw: float) -> str:
    """
    直接传入坐标导航（不污染 YAML）。

    通过 autopatrol.launch.py 的 target_coordinates 参数。
    """
    global _active_nav_process, _active_nav_target

    with _global_nav_lock:
        _blk = _blocked_by_follow_msg()
        if _blk:
            return _blk
        loc_blk = _localization_block_msg()
        if loc_blk:
            return loc_blk
        preempted = _preempt_active_nav()

        fishbot_ws = _get_fishbot_ws()
        cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {fishbot_ws}/install/setup.bash && "
            f"ros2 launch autopatrol_robot autopatrol.launch.py "
            f"target_coordinates:='{x},{y},{yaw}'"
        )

        _reap_orphan_nav_processes()
        _active_nav_process = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        _active_nav_target = f"({x:.1f},{y:.1f})"
        new_target = _active_nav_target

    if preempted:
        _notify_callbacks(
            "preempted",
            f"🛑 已中断「{preempted}」，改为前往坐标 {new_target}")

    threading.Thread(target=_monitor_process, daemon=True).start()
    return f"📍 正在前往坐标 {new_target}..."


def cancel_navigation() -> str:
    """取消当前 CLI 导航（kill 整个进程组）。"""
    global _active_nav_process, _active_nav_target

    with _global_nav_lock:
        if _active_nav_process is None:
            return "ℹ️ 没有进行中的导航任务"
        if _active_nav_process.poll() is not None:
            rc = _active_nav_process.returncode
            proc = _active_nav_process
            _active_nav_process = None
            _active_nav_target = ""
            _cleanup_process(proc)
            return f"ℹ️ 上次导航已结束（退出码 {rc}）"

        pid = _active_nav_process.pid
        target = _active_nav_target

    # 先优雅终止（SIGINT 模拟 Ctrl+C）
    try:
        os.killpg(os.getpgid(pid), signal.SIGINT)
        time.sleep(1.5)
    except Exception:
        pass

    # 如果还在运行，强制杀死
    with _global_nav_lock:
        if _active_nav_process and _active_nav_process.poll() is None:
            try:
                os.killpg(os.getpgid(_active_nav_process.pid), signal.SIGKILL)
            except Exception:
                pass
            proc = _active_nav_process
            _active_nav_process = None
            _active_nav_target = ""
            _cleanup_process(proc)

    return f"🛑 已取消导航（目标：{target}）"


def get_nav_status() -> str:
    """查询 CLI 导航状态。"""
    global _active_nav_process, _active_nav_target
    with _global_nav_lock:
        if _active_nav_process is None:
            return "🔵 没有进行中的 CLI 导航任务"
        rc = _active_nav_process.poll()
        if rc is None:
            return f"🟢 正在导航到 {_active_nav_target}..."
        elif rc == 0:
            proc = _active_nav_process
            _active_nav_process = None
            _active_nav_target = ""
            _cleanup_process(proc)
            return f"✅ 上次导航已完成（{_active_nav_target}）"
        else:
            proc = _active_nav_process
            _active_nav_process = None
            _active_nav_target = ""
            _cleanup_process(proc)
            return f"❌ 上次导航失败（{_active_nav_target}，退出码 {rc}）"


def wait_for_nav_completion(timeout: float = 600.0) -> tuple:
    """
    阻塞等待当前 CLI 导航进程结束。

    供序列导航使用 — launch 启动子进程后调用此函数等待完成，
    再启动下一个。通过轮询 _active_nav_process.poll() 检测退出，
    与 _monitor_process 后台线程不冲突（两者都检查 poll，先到先清理）。

    Args:
        timeout: 超时秒数（默认 600 秒 = 10 分钟）

    Returns:
        (success: bool, message: str)
    """
    global _active_nav_process, _active_nav_target

    proc = None
    target = ""
    with _global_nav_lock:
        proc = _active_nav_process
        target = _active_nav_target or ""

    if proc is None:
        return True, "无进行中的导航"

    deadline = time.time() + timeout
    while time.time() < deadline:
        rc = proc.poll()
        if rc is not None:
            with _global_nav_lock:
                if _active_nav_process is proc:
                    _active_nav_process = None
                    _active_nav_target = ""
            _cleanup_process(proc)
            if rc == 0:
                return True, f"导航到{target}完成"
            else:
                return False, f"导航到{target}失败（退出码 {rc}）"
        time.sleep(0.5)

    # 超时 → 强制 kill
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception:
        pass
    with _global_nav_lock:
        if _active_nav_process is proc:
            _active_nav_process = None
            _active_nav_target = ""
    _cleanup_process(proc)
    return False, f"导航到{target}超时（>{timeout:.0f}秒）"


# ============================================================
# 异步导航（非阻塞版本，供 AI 工具使用）
# ------------------------------------------------------------
# ★ 2026-11-01：**彻底删除「导航完成后自动返回起点」流程**
#   （用户确认：「每一次导航完成后调返回起点的流程可以去掉」）
#
#   原实现在这里挂了一条后台 worker，到达后
#   自动 launch_return_to_start()。后果是用户只说「去厕所」，车到了
#   之后又自己跑回起点。而错误消息里还会带出旧目标名（"回起点"），
#   把 3B 模型也带偏 —— 详见 _preempt_active_nav 那段复盘。
#
#   现在只做一件事：启动导航 → 后台盯着进程 → 到达就通知完事。
#   需要回起点时，用户说「回起点」调 return_to_start 工具即可。
# ============================================================

# ============================================================
# "目标就在脚下" 检查（2026-11-01）
# ------------------------------------------------------------
# 实机现象：用户从起点说"去厕所"，车一动不动，日志却报
# 「[NavPhase] arrived: ✅ 已到达 厕所！」。
#
# 查明原因是**坐标录错了**，不是导航坏了：
#     起点 initial_point = (0.0000,  0.0000)
#     厕所 custom_points = (0.0557, -0.0023)   ← 相距 5.6 厘米
# nav2 的 xy_goal_tolerance 默认 0.25 米，目标一进去就判定成功。
#
# 与其让用户看到"已到达"却车没动（以为系统骗人），不如直接说清楚：
# 这个点就在你脚下，八成是录点时机器人没真正站在那儿。
# ============================================================
_GOAL_AT_FEET_M = 0.30


def _goal_at_feet_msg(name: str, tx: float, ty: float) -> Optional[str]:
    """目标点就在机器人当前位置附近时，返回提示文案；否则返回 None。

    任何异常都返回 None（宁可放行导航，也不要因为检查本身出错而卡死）。
    """
    try:
        # 延迟导入：navigation_service 顶层就 import 了本模块，会循环依赖
        from autopatrol_robot.navigation_service import get_global_nav_service

        svc = get_global_nav_service()
        if svc is None:
            return None
        pos, _reason = svc.get_current_position(detailed=True)
        if pos is None:
            return None
        dist = ((tx - pos['x']) ** 2 + (ty - pos['y']) ** 2) ** 0.5
    except Exception:                                  # noqa: BLE001
        return None

    if dist > _GOAL_AT_FEET_M:
        return None

    # 区分两种"原地不动"，否则提示文不对题：
    #   A. 目标本身就贴着地图原点 → 大概率录点录错了
    #   B. 目标在远处，但车已经在那儿了 → 只是不需要再走
    dist_home = (tx * tx + ty * ty) ** 0.5
    if dist_home < _GOAL_AT_FEET_M:
        advice = (
            f"　「{name}」的坐标 ({tx:.3f}, {ty:.3f}) 几乎就在地图原点，"
            f"而起点 initial_point 也在原点附近 —— 多半是保存「{name}」时"
            f"机器人并没有真的站在那个位置，或者当时的 AMCL 位姿是错的。\n"
            f"　解决办法：把车开到真正的「{name}」，再说一次"
            f"「这里是{name}」重新保存即可。"
        )
    else:
        advice = (
            f"　车现在已经在这个坐标上了，所以不需要再移动。\n"
            f"　如果你其实希望车去别的地方，请先确认 AMCL 位姿准确"
            f"（看网页顶部的定位状态条），或者换一个目标点。"
        )

    return (
        f"⚠️ 「{name}」就在车旁边（相距 {dist * 100:.0f} 厘米），"
        f"不用导航也已经站在目标点上了，所以没有启动导航。\n"
        f"　目标坐标: ({tx:.3f}, {ty:.3f})\n"
        f"　当前坐标: ({pos['x']:.3f}, {pos['y']:.3f})\n"
        + advice
    )


def launch_navigate_with_return_async(index: int, name: str) -> str:
    """启动「导航到目标」的异步导航（非阻塞，**不自动返航**）。

      1. 立即启动导航 subprocess
      2. 返回启动确认字符串（如 "📍 正在前往厨房..."）
      3. 后台线程监控导航完成 → 通知回调

    阶段回调：
      'started'   → 导航已启动
      'arrived'   → 到达目标
      'failed'    → 失败

    Args:
        index: 航点索引（1-based）
        name: 目标名称

    Returns:
        启动确认字符串
    """
    # 目标就在脚下时直接拒绝 —— 否则 nav2 会瞬间返回 success，
    # 用户看到"已到达"却一步没走，比报错更让人困惑。
    try:
        from autopatrol_robot.location_store import get_location_store
        _store = get_location_store()
        _coords = _store.get_coords(name) or _store.get_predefined_coords(index)
        if _coords:
            _msg = _goal_at_feet_msg(name, _coords[0], _coords[1])
            if _msg:
                return _msg
    except Exception:                                  # noqa: BLE001
        pass

    # silent=True：本流程由 worker 统一上报阶段，monitor 再报一次会变成
    # 「completed: 导航完成：厕所」+「arrived: 已到达 厕所！」两条重复播报。
    result = launch_navigate_by_index(index, name, silent=True)
    if "⚠️" in result:
        return f"❌ 启动导航失败：{result}"

    _notify_callbacks("started", f"📍 正在前往 {name}...")

    threading.Thread(
        target=_async_navigate_worker,
        args=(name,),
        daemon=True,
    ).start()

    return f"📍 正在前往 {name}..."


def launch_navigate_sequence_async(indices: list, names: str = "") -> str:
    """异步序列导航（非阻塞，**不自动返航**）。

    patrol_node 内部按序执行所有目标：
      * 中间航点到达后**阻塞等按钮确认**（期间位姿被 /odom_freeze 冻住）
      * 最后一个航点到达即结束
      · 任一航点等按钮超时 → **取消整个任务**

    阶段回调：
      'started'   → 序列导航已启动
      'arrived'   → 序列中所有目标已完成
      'failed'    → 任一阶段失败

    Args:
        indices: 航点索引列表（1-based）
        names: 目标名称描述

    Returns:
        启动确认字符串
    """
    result = launch_navigate_sequence(indices, names, silent=True)
    if "⚠️" in result:
        return f"❌ 启动序列导航失败：{result}"

    _notify_callbacks("started", f"📍 序列导航开始：{names or indices}")

    threading.Thread(
        target=_async_sequence_worker,
        args=(names or str(indices),),
        daemon=True,
    ).start()

    return f"📍 序列导航开始：{names or indices}"


# ============================================================
# 异步 worker：只负责「盯着导航进程 + 上报结果」
# ------------------------------------------------------------
# ★ 2026-11-01：删掉了原来的「到达后自动返回起点」第二/第三阶段。
#   用户确认这个流程要去掉 —— 只说「去厕所」，车到了就该停在那儿。
# ============================================================

def _async_navigate_worker(target_name: str):
    """后台等待导航完成并上报结果（不返航）。"""
    _async_wait_and_report(target_name, timeout=600, label='')


def _async_sequence_worker(target_name: str):
    """后台等待**序列**导航完成并上报结果（不返航）。

    序列导航内部会依次走完所有航点（中间点等按钮确认），
    所以这里的 timeout 给得更宽。
    """
    _async_wait_and_report(target_name, timeout=1800, label='序列导航')


def _async_wait_and_report(target_name: str, timeout: float,
                           label: str = ''):
    """等待当前活动导航进程结束并发出回调。"""
    global _active_nav_process, _active_nav_target

    with _global_nav_lock:
        proc = _active_nav_process
        cur_target = _active_nav_target
    if proc is None:
        _notify_callbacks("failed", "❌ 导航进程意外消失")
        return
    if cur_target and target_name and cur_target != target_name:
        print("[导航] %sworker 发现当前任务是「%s」而非「%s」，已退出"
              % (label, cur_target, target_name))
        return

    try:
        proc.wait(timeout=timeout)
        rc = proc.returncode
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
        rc = -9

    with _global_nav_lock:
        if _active_nav_process is proc:
            _active_nav_process = None
            _active_nav_target = ""
    _cleanup_process(proc)

    if rc != 0:
        # 用户改主意导致的中断不是失败，静默退出（新任务的回调已在跑）
        if proc.pid in _preempted_pids:
            _preempted_pids.discard(proc.pid)
            print("[导航] %s「%s」被新指令抢占，跳过失败通知"
                  % (label, target_name))
            return
        _notify_callbacks("failed",
                          f"❌ {label}前往{target_name}失败（退出码 {rc}）")
        return

    _notify_callbacks("arrived",
                      f"✅ {label}已到达 {target_name}！")


def get_global_nav_lock() -> threading.Lock:
    """获取全局导航互斥锁（供 NavigationService 使用）。"""
    return _global_nav_lock
