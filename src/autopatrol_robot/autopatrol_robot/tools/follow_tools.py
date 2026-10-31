"""
tools/follow_tools.py
AI 可调用的看护模式/跟随模式工具。
"""

import os
import signal
import subprocess
import threading
import time
from typing import Optional

from langchain_core.tools import tool

# ============================================================
# 跟随模式进程管理
# ============================================================
_follow_process: Optional[subprocess.Popen] = None
_follow_process_lock = threading.Lock()


def get_follow_process() -> Optional[subprocess.Popen]:
    global _follow_process
    return _follow_process


def set_follow_process(proc):
    global _follow_process
    with _follow_process_lock:
        _follow_process = proc


def is_follow_mode_running() -> bool:
    global _follow_process
    with _follow_process_lock:
        if _follow_process is None:
            return False
        if _follow_process.poll() is not None:
            _follow_process = None
            return False
        return True


def stop_follow_mode_process(force: bool = False) -> bool:
    """
    停止跟随模式的进程（递归杀死所有子进程）。

    Returns:
        是否成功停止
    """
    global _follow_process
    with _follow_process_lock:
        if _follow_process is None:
            return True
        try:
            if _follow_process.poll() is not None:
                _follow_process = None
                return True
            import psutil
            parent = psutil.Process(_follow_process.pid)
            children = parent.children(recursive=True)
            all_procs = [parent] + children
            print(f"[跟随模式] 找到 {len(all_procs)} 个相关进程，正在停止...")
            if not force:
                for proc in all_procs:
                    try:
                        proc.send_signal(signal.SIGINT)
                    except Exception:
                        pass
                time.sleep(1)
            still_alive = [p for p in all_procs if _try_is_running(p)]
            if still_alive:
                print(f"[跟随模式] {len(still_alive)} 个进程未响应，强制杀死...")
                for proc in still_alive:
                    try:
                        proc.kill()
                    except Exception:
                        pass
                time.sleep(0.5)
            _follow_process = None
            print("[跟随模式] 所有跟随进程已清理")
            return True
        except Exception as e:
            print(f"[跟随模式] 停止进程失败: {e}")
            _follow_process = None
            return False


def _try_is_running(proc) -> bool:
    try:
        return proc.is_running()
    except Exception:
        return False


def _get_detection_ws() -> str:
    # 原先默认值是原作者机器的 '/home/ke/detection_ws'。
    # 统一由 paths.py 定位；本机没有该工作空间时，
    # 看护/跟随模式会提示路径不存在，其余功能不受影响。
    from autopatrol_robot.paths import detection_ws
    return detection_ws()


# ============================================================
# 工具定义
# ============================================================

@tool
def start_patrol_mode() -> str:
    """
    开启看护模式/跟随模式，启动 ESP32 摄像头追踪节点。

    当用户说"开启看护模式"、"启动跟随"、"开始看护"时使用此工具。
    """
    global _follow_process

    log_file = "/tmp/patrol_mode_debug.log"

    def write_log(msg):
        with open(log_file, 'a') as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
        print(f"[看护模式] {msg}")

    write_log("=== 开始启动看护模式 ===")

    if is_follow_mode_running():
        write_log("看护模式已在运行中")
        return "⚠️ 看护模式已经在运行中"

    # 互斥：雷达跟随正在跑的话先停掉，否则两个节点抢 /cmd_vel，
    # 车会在原地抽搐（本项目没有 twist_mux）。
    try:
        from autopatrol_robot.tools.lidar_follow_tools import (
            is_lidar_follow_running, stop_lidar_follow_process)
        if is_lidar_follow_running():
            write_log("检测到雷达跟随在运行，先停止它（互斥）")
            stop_lidar_follow_process(force=False)
            time.sleep(1.0)
    except Exception as e:                          # noqa: BLE001
        write_log("检查雷达跟随状态时出错（忽略）: %s" % e)

    from autopatrol_robot.paths import fishbot_ws as _get_fishbot_ws
    fishbot_ws = _get_fishbot_ws()
    detection_ws = _get_detection_ws()

    if not os.path.exists(detection_ws):
        write_log(f"错误: detection_ws 不存在: {detection_ws}")
        return f"❌ 检测工作空间不存在: {detection_ws}"

    # 查找 launch 文件
    launch_file = None
    candidates = [
        f"{detection_ws}/src/esp32_cam_detector/launch/follow.launch.py",
        f"{detection_ws}/install/esp32_cam_detector/share/esp32_cam_detector/launch/follow.launch.py",
    ]
    for cand in candidates:
        if os.path.exists(cand):
            launch_file = cand
            break

    if not launch_file:
        write_log("未找到 follow.launch.py")
        return f"❌ 找不到 follow.launch.py。\n期望路径: {candidates[0]}"

    command = (
        f"export ROS_DOMAIN_ID=0 && "
        f"source /opt/ros/humble/setup.bash && "
        f"source {fishbot_ws}/install/setup.bash && "
        f"source {detection_ws}/install/setup.bash && "
        f"ros2 launch {launch_file}"
    )
    write_log(f"命令: {command}")
    open(log_file, 'w').close()

    try:
        process = subprocess.Popen(
            ['bash', '-c', command],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
            cwd="/tmp",
        )
        write_log(f"进程 PID: {process.pid}")

        def read_output():
            for line in iter(process.stdout.readline, ''):
                if line:
                    print(f"[跟随模式] {line.rstrip()}")
                    with open(log_file, 'a') as f:
                        f.write(line)
            process.stdout.close()

        output_thread = threading.Thread(target=read_output, daemon=True)
        output_thread.start()

        time.sleep(5)
        poll_result = process.poll()
        write_log(f"进程 poll 结果: {poll_result}")

        if poll_result is not None:
            with open(log_file, 'r') as f:
                lines = f.readlines()
                error_tail = ''.join(lines[-20:])
            return (
                f"❌ 看护模式启动失败 (退出码 {poll_result})。\n"
                f"日志最后几行:\n{error_tail}\n完整日志: {log_file}"
            )

        set_follow_process(process)
        write_log("进程启动成功")

        def monitor():
            returncode = process.wait()
            write_log(f"监控: 进程退出，退出码 {returncode}")
            with _follow_process_lock:
                nonlocal_local = _follow_process
                if nonlocal_local == process:
                    set_follow_process(None)
                    write_log("已清空进程引用")

        threading.Thread(target=monitor, daemon=True).start()

        return (
            f"✅ 看护模式已启动！\n"
            f"进程 PID: {process.pid}\n"
            f"日志文件: {log_file}\n"
            f"使用 '关闭看护模式' 停止。"
        )

    except Exception as e:
        write_log(f"异常: {type(e).__name__}: {str(e)}")
        import traceback
        write_log(traceback.format_exc())
        return f"❌ 启动异常: {str(e)}\n详情见日志: {log_file}"


@tool
def stop_patrol_mode() -> str:
    """
    关闭看护模式/跟随模式，停止 ESP32 摄像头追踪节点。

    当用户说"关闭看护模式"、"停止跟随"时使用此工具。
    """
    if not is_follow_mode_running():
        return "ℹ️ 看护模式未在运行中"

    print(f"[看护模式] 正在停止跟随节点...")

    if stop_follow_mode_process(force=False):
        time.sleep(1)
        if not is_follow_mode_running():
            return "✅ 看护模式已关闭，摄像头跟随已停止"
        else:
            stop_follow_mode_process(force=True)
            return "✅ 看护模式已强制关闭"
    else:
        return "❌ 关闭看护模式失败，请稍后重试"


@tool
def get_patrol_mode_status() -> str:
    """
    查询看护模式/跟随模式的运行状态。

    当用户问"看护模式开了吗"、"跟随状态怎样"时使用此工具。
    """
    if is_follow_mode_running():
        return "🟢 看护模式正在运行中，摄像头正在追踪人物"
    else:
        return "🔴 看护模式未运行"
