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


def _monitor_process():
    """后台监控线程：等待 CLI 进程结束并清理状态。结束时会通知回调。"""
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

    # 通知回调
    if rc == 0:
        _notify_callbacks("completed", f"✅ 导航完成：{target}")
    else:
        _notify_callbacks("failed", f"❌ 导航失败：{target}（退出码 {rc}）")


# ============================================================
# Public API
# ============================================================

def launch_navigate_by_index(index: int, target_name: str = "") -> str:
    """
    通过航点索引启动 CLI 导航（非阻塞，1-based）。

    Args:
        index: 航点索引（1-based，与 autopatrol.launch.py 的 target_indices 一致）
        target_name: 目标名称（用于日志和状态显示）

    Returns:
        结果描述字符串
    """
    global _active_nav_process, _active_nav_target

    with _global_nav_lock:
        _blk = _blocked_by_follow_msg()
        if _blk:
            return _blk
        if _active_nav_process and _active_nav_process.poll() is None:
            return f"⚠️ 已有导航任务进行中（目标：{_active_nav_target}），请先取消"

        fishbot_ws = _get_fishbot_ws()
        cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {fishbot_ws}/install/setup.bash && "
            f"ros2 launch autopatrol_robot autopatrol.launch.py "
            f"target_indices:={index}"
        )

        _active_nav_process = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        _active_nav_target = target_name or f"点{index}"

    # 启动监控线程
    threading.Thread(target=_monitor_process, daemon=True).start()

    return f"📍 正在前往 {_active_nav_target}..."


def launch_navigate_sequence(indices: list, target_names: str = "") -> str:
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
        if _active_nav_process and _active_nav_process.poll() is None:
            return f"⚠️ 已有导航任务进行中（目标：{_active_nav_target}），请先取消"

        indices_str = ','.join(str(i) for i in indices)
        fishbot_ws = _get_fishbot_ws()
        cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {fishbot_ws}/install/setup.bash && "
            f"ros2 launch autopatrol_robot autopatrol.launch.py "
            f"target_indices:={indices_str}"
        )

        _active_nav_process = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        _active_nav_target = target_names or f"序列{indices_str}"

    threading.Thread(target=_monitor_process, daemon=True).start()
    return f"📍 序列导航开始：{_active_nav_target}"


def launch_return_to_start() -> str:
    """
    启动返航到起点 —— 使用 fishbot_application 的 nav_to_pose 节点。

    与 launch_navigate_by_index 一致：通过 CLI 子进程运行，
    受全局互斥锁保护，与 Web UI 导航互斥。
    """
    global _active_nav_process, _active_nav_target

    with _global_nav_lock:
        _blk = _blocked_by_follow_msg()
        if _blk:
            return _blk
        if _active_nav_process and _active_nav_process.poll() is None:
            return f"⚠️ 已有导航任务进行中（目标：{_active_nav_target}），请先取消"

        fishbot_ws = _get_fishbot_ws()
        cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {fishbot_ws}/install/setup.bash && "
            f"ros2 run fishbot_application nav_to_pose"
        )

        _active_nav_process = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        _active_nav_target = "回起点"

    threading.Thread(target=_monitor_process, daemon=True).start()
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
        if _active_nav_process and _active_nav_process.poll() is None:
            return f"⚠️ 已有导航任务进行中，请先取消"

        fishbot_ws = _get_fishbot_ws()
        cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {fishbot_ws}/install/setup.bash && "
            f"ros2 launch autopatrol_robot autopatrol.launch.py "
            f"target_coordinates:='{x},{y},{yaw}'"
        )

        _active_nav_process = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        _active_nav_target = f"({x:.1f},{y:.1f})"

    threading.Thread(target=_monitor_process, daemon=True).start()
    return f"📍 正在前往坐标 {_active_nav_target}..."


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
# 异步导航+返回（非阻塞版本，供 AI 工具使用）
# ============================================================

def launch_navigate_with_return_async(index: int, name: str,
                                       return_to_start: bool = True) -> str:
    """
    启动"前往目标 + 自动返回起点"异步导航（非阻塞）。

    与 _navigate_to_and_return 的阻塞版本不同，此函数：
      1. 立即启动导航 subprocess
      2. 返回启动确认字符串（如 "📍 正在前往厨房..."）
      3. 后台线程监控导航完成 → 通知回调 → 启动返航 → 再通知

    阶段回调顺序：
      'started'   → 导航已启动
      'arrived'   → 到达目标（nav 完成）
      'returning' → 正在返回起点
      'completed' → 已回到起点
      'failed'    → 任一阶段失败

    Args:
        index: 航点索引（1-based）
        name: 目标名称
        return_to_start: 是否到达后自动返回起点

    Returns:
        启动确认字符串
    """
    # Step 1: Launch nav to target
    result = launch_navigate_by_index(index, name)
    if "⚠️" in result:
        return f"❌ 启动导航失败：{result}"

    _notify_callbacks("started", f"📍 正在前往 {name}...")

    # Step 2: Start background thread for monitoring + return
    threading.Thread(
        target=_async_navigate_and_return_worker,
        args=(name, return_to_start),
        daemon=True,
    ).start()

    return f"📍 正在前往 {name}..."


def launch_navigate_sequence_async(indices: list, names: str = "",
                                    return_to_start: bool = True) -> str:
    """
    异步序列导航（非阻塞）+ 可选自动返航。

    patrol_node 内部按序执行所有目标，全部完成后：
      - return_to_start=True  → 自动返航到起点
      - return_to_start=False → 停在最后一个目标点

    阶段回调：
      'started'   → 序列导航已启动
      'arrived'   → 序列中所有目标已完成
      'returning' → 正在返回起点（仅 return_to_start=True）
      'completed' → 已回到起点
      'failed'    → 任一阶段失败

    Args:
        indices: 航点索引列表（1-based）
        names: 目标名称描述
        return_to_start: 序列完成后是否自动返回起点

    Returns:
        启动确认字符串
    """
    result = launch_navigate_sequence(indices, names)
    if "⚠️" in result:
        return f"❌ 启动序列导航失败：{result}"

    _notify_callbacks("started", f"📍 序列导航开始：{names or indices}")

    # 启动后台线程：等待序列完成 → 可选返航
    threading.Thread(
        target=_async_sequence_and_return_worker,
        args=(names or str(indices), return_to_start),
        daemon=True,
    ).start()

    return f"📍 序列导航开始：{names or indices}"


def _async_navigate_and_return_worker(target_name: str, return_to_start: bool):
    """
    后台工作线程：等待导航完成 → 可选返航 → 通知回调。

    通过轮询 poll() 检测 subprocess 退出，与 _monitor_process 不冲突。
    """
    global _active_nav_process, _active_nav_target

    # --- 等待第一阶段完成（前往目标） ---
    proc = _active_nav_process
    if proc is None:
        _notify_callbacks("failed", f"❌ 导航进程意外消失")
        return

    try:
        proc.wait(timeout=600)
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
        _notify_callbacks("failed",
                          f"❌ 前往{target_name}失败（退出码 {rc}）")
        return

    _notify_callbacks("arrived", f"✅ 已到达 {target_name}！")

    # --- 第二阶段：返回起点（可选） ---
    if not return_to_start:
        _notify_callbacks("completed", f"✅ 到达{target_name}（不返回起点）")
        return

    time.sleep(0.5)  # 短暂间隙

    ret_result = launch_return_to_start()
    if "⚠️" in ret_result:
        _notify_callbacks("failed",
                          f"❌ 返航启动失败：{ret_result}")
        return

    _notify_callbacks("returning", "📍 正在返回起点...")

    # 等待返航完成
    proc2 = _active_nav_process
    if proc2 is None:
        _notify_callbacks("failed", "❌ 返航进程意外消失")
        return

    try:
        proc2.wait(timeout=600)
        rc2 = proc2.returncode
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc2.pid), signal.SIGKILL)
        except Exception:
            pass
        rc2 = -9

    with _global_nav_lock:
        if _active_nav_process is proc2:
            _active_nav_process = None
            _active_nav_target = ""
    _cleanup_process(proc2)

    if rc2 == 0:
        _notify_callbacks("completed",
                          f"✅ 已回到起点！（{target_name}导航完成）")
    else:
        _notify_callbacks("failed",
                          f"❌ 返航失败（退出码 {rc2}）")


def _async_sequence_and_return_worker(target_name: str, return_to_start: bool):
    """
    后台工作线程：等待序列导航完成 → 可选返航 → 通知回调。

    与 _async_navigate_and_return_worker 结构一致，
    区别在于第一阶段等待的是序列导航子进程（target_indices=2,3,...）。
    """
    global _active_nav_process, _active_nav_target

    # --- 第一阶段：等待序列完成 ---
    proc = _active_nav_process
    if proc is None:
        _notify_callbacks("failed", "❌ 序列导航进程意外消失")
        return

    try:
        proc.wait(timeout=900)  # 序列导航超时更长
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
        _notify_callbacks("failed",
                          f"❌ 序列导航{target_name}失败（退出码 {rc}）")
        return

    _notify_callbacks("arrived", f"✅ 序列导航{target_name}已完成！")

    # --- 第二阶段：返回起点（可选） ---
    if not return_to_start:
        _notify_callbacks("completed", f"✅ 序列{target_name}完成（不返回起点）")
        return

    time.sleep(0.5)

    ret_result = launch_return_to_start()
    if "⚠️" in ret_result:
        _notify_callbacks("failed",
                          f"❌ 返航启动失败：{ret_result}")
        return

    _notify_callbacks("returning", "📍 正在返回起点...")

    proc2 = _active_nav_process
    if proc2 is None:
        _notify_callbacks("failed", "❌ 返航进程意外消失")
        return

    try:
        proc2.wait(timeout=600)
        rc2 = proc2.returncode
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc2.pid), signal.SIGKILL)
        except Exception:
            pass
        rc2 = -9

    with _global_nav_lock:
        if _active_nav_process is proc2:
            _active_nav_process = None
            _active_nav_target = ""
    _cleanup_process(proc2)

    if rc2 == 0:
        _notify_callbacks("completed",
                          f"✅ 已回到起点！（序列{target_name}完成）")
    else:
        _notify_callbacks("failed",
                          f"❌ 返航失败（退出码 {rc2}）")


def get_global_nav_lock() -> threading.Lock:
    """获取全局导航互斥锁（供 NavigationService 使用）。"""
    return _global_nav_lock
