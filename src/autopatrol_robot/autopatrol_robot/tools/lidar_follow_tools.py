"""
tools/lidar_follow_tools.py
AI 可调用的雷达跟随工具。

跟随功能本身在独立包 fishbot_lidar_follow 里：
    节点   fishbot_lidar_follow/lidar_follow.py
    启动   ros2 launch fishbot_lidar_follow lidar_follow.launch.py
    仿真   ros2 run fishbot_lidar_follow lidar_follow_sim

本文件只负责「AI 怎么启动/停止它」，不包含控制逻辑。

与 follow_tools.py（视觉看护模式）的区别：
    follow_tools   → ESP32 摄像头人体骨架跟踪
    lidar_follow   → 单线激光雷达测距跟随

两者都会往 /cmd_vel 发指令，所以**互斥**：
开启雷达跟随时会先停掉视觉看护，反之亦然。
"""

import os
import signal
import subprocess
import threading
import time
from typing import Optional

from langchain_core.tools import tool

# ============================================================
# 进程管理
# ============================================================
_follow_process: Optional[subprocess.Popen] = None
_follow_lock = threading.Lock()
_LOG_FILE = '/tmp/lidar_follow_debug.log'

# 与 fishbot_lidar_follow.mutual_exclusion.LOCK_FILE 保持一致
_LOCK_FILE = '/tmp/fishbot_lidar_follow.lock'

# 启动时用到的包名，方便和节点/导航互斥模块对齐
LAUNCH_PKG = 'fishbot_lidar_follow'
LAUNCH_FILE = 'lidar_follow.launch.py'


def _write_log(msg):
    try:
        with open(_LOG_FILE, 'a') as f:
            f.write('%s - %s\n' % (time.strftime('%Y-%m-%d %H:%M:%S'), msg))
    except Exception:                               # noqa: BLE001
        pass
    print('[雷达跟随] %s' % msg)


def get_lidar_follow_process() -> Optional[subprocess.Popen]:
    global _follow_process
    return _follow_process


def is_lidar_follow_running() -> bool:
    """跟随节点是否在运行。

    除了看进程，还检查锁文件 —— 节点可能被 launch 或手动启动，
    这时 _follow_process 是 None，但锁文件存在说明它真在跑。
    两条线索结合，避免「明明在跑却说没开」。
    """
    global _follow_process
    with _follow_lock:
        if _follow_process is not None:
            if _follow_process.poll() is None:
                return True
            _follow_process = None

    # 进程记录丢了，看锁文件 + 实际进程
    if os.path.exists(_LOCK_FILE):
        try:
            out = subprocess.run(
                ['pgrep', '-f', 'lidar_follow'],
                capture_output=True, text=True, timeout=3)
            return bool(out.stdout.strip())
        except Exception:                           # noqa: BLE001
            return True
    return False


def stop_lidar_follow_process(force: bool = False) -> bool:
    """停止跟随节点。先让它自己安全停车，再兜底 kill。"""
    global _follow_process
    with _follow_lock:
        proc = _follow_process
        _follow_process = None

    stopped = False

    # 1) 先尝试优雅退出（节点 finally 里会发零速度）
    if proc is not None and proc.poll() is None:
        try:
            import psutil
            parent = psutil.Process(proc.pid)
            for p in [parent] + parent.children(recursive=True):
                try:
                    p.send_signal(signal.SIGINT)
                except Exception:                   # noqa: BLE001
                    pass
            stopped = True
        except Exception:                           # noqa: BLE001
            try:
                proc.send_signal(signal.SIGINT)
                stopped = True
            except Exception:                       # noqa: BLE001
                pass

    # 2) 独立启动的（锁文件在但没进程记录）
    if not stopped:
        try:
            out = subprocess.run(
                ['pgrep', '-f', 'lidar_follow'],
                capture_output=True, text=True, timeout=3)
            pids = [int(x) for x in out.stdout.split() if x.isdigit()]
            for pid in pids:
                if pid == os.getpid():
                    continue
                try:
                    os.kill(pid, signal.SIGINT)
                    stopped = True
                except Exception:                   # noqa: BLE001
                    pass
        except Exception:                           # noqa: BLE001
            pass

    time.sleep(1.2)

    # 3) 还没退就强杀
    try:
        out = subprocess.run(
            ['pgrep', '-f', 'lidar_follow'],
            capture_output=True, text=True, timeout=3)
        alive = [int(x) for x in out.stdout.split() if x.isdigit()]
        if alive:
            _write_log('%d 个进程未响应 SIGINT，强制 kill' % len(alive))
            for pid in alive:
                if pid == os.getpid():
                    continue
                try:
                    os.kill(pid, signal.SIGKILL)
                except Exception:                   # noqa: BLE001
                    pass
            stopped = True
    except Exception:                               # noqa: BLE001
        pass

    # 4) 清掉锁文件，否则导航会一直被拒
    try:
        if os.path.exists(_LOCK_FILE):
            os.remove(_LOCK_FILE)
    except Exception:                               # noqa: BLE001
        pass

    return stopped


# ============================================================
# 工具定义
# ============================================================
@tool
def start_lidar_follow() -> str:
    """开启雷达跟随：小车会跟着前方 40cm 内出现的障碍物移动，保持约 50cm 距离。

    当用户说"开始雷达跟随"、"用雷达跟着我"、"跟着我走"、"雷达跟随"时使用。
    注意：这是用激光雷达测距跟随，不是摄像头；与「看护模式」互斥。
    """
    global _follow_process

    _write_log('=== 请求启动雷达跟随 ===')
    open(_LOG_FILE, 'w').close()

    if is_lidar_follow_running():
        _write_log('已在运行')
        return '⚠️ 雷达跟随已经在运行中'

    # 互斥：先停视觉看护，否则两个节点抢 /cmd_vel
    try:
        from autopatrol_robot.tools.follow_tools import (
            is_follow_mode_running, stop_follow_mode_process)
        if is_follow_mode_running():
            _write_log('检测到视觉看护模式在运行，先停止它（互斥）')
            stop_follow_mode_process(force=False)
            time.sleep(1.0)
    except Exception as e:                          # noqa: BLE001
        _write_log('检查视觉看护模式时出错（忽略）: %s' % e)

    from autopatrol_robot.paths import fishbot_ws as _get_fishbot_ws
    fishbot_ws = _get_fishbot_ws()

    if not os.path.exists(os.path.join(fishbot_ws, 'install', 'setup.bash')):
        return '❌ 工作空间未编译，请先 colcon build'

    # 跟随功能在独立包里，没编译就给出明确指引，
    # 否则 ros2 launch 只会甩一句 "Package not found"
    pkg_share = os.path.join(
        fishbot_ws, 'install', LAUNCH_PKG, 'share', LAUNCH_PKG, 'launch',
        LAUNCH_FILE)
    if not os.path.exists(pkg_share):
        _write_log('未找到 %s' % pkg_share)
        return (
            '❌ 雷达跟随包 %s 未编译。\n'
            '请在终端执行：\n'
            '    cd %s && source /opt/ros/humble/setup.bash && '
            'colcon build --packages-select %s --symlink-install'
            % (LAUNCH_PKG, fishbot_ws, LAUNCH_PKG))

    command = (
        'export ROS_DOMAIN_ID=0 && '
        'source /opt/ros/humble/setup.bash && '
        'source %s/install/setup.bash && '
        'exec ros2 launch %s %s'
    ) % (fishbot_ws, LAUNCH_PKG, LAUNCH_FILE)
    _write_log('命令: %s' % command)

    try:
        proc = subprocess.Popen(
            ['bash', '-c', command],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
            cwd='/tmp',
        )
        _write_log('PID: %d' % proc.pid)

        def _reader():
            try:
                for line in iter(proc.stdout.readline, ''):
                    if line:
                        with open(_LOG_FILE, 'a') as f:
                            f.write(line)
            except Exception:                       # noqa: BLE001
                pass
            finally:
                try:
                    proc.stdout.close()
                except Exception:                   # noqa: BLE001
                    pass

        threading.Thread(target=_reader, daemon=True).start()

        # 等一下看有没有立刻挂掉（最常见的失败是 /scan 没数据）
        time.sleep(4.0)
        rc = proc.poll()
        if rc is not None:
            tail = ''
            try:
                with open(_LOG_FILE, 'r') as f:
                    tail = ''.join(f.readlines()[-15:])
            except Exception:                       # noqa: BLE001
                pass
            return ('❌ 雷达跟随启动失败（退出码 %s）。\n'
                    '常见原因：/scan 没有数据、或雷达未启动。\n'
                    '日志最后几行:\n%s\n完整日志: %s'
                    % (rc, tail, _LOG_FILE))

        with _follow_lock:
            _follow_process = proc

        def _monitor():
            rc2 = proc.wait()
            _write_log('进程退出，码 %s' % rc2)
            with _follow_lock:
                global _follow_process
                if _follow_process == proc:
                    _follow_process = None

        threading.Thread(target=_monitor, daemon=True).start()

        return (
            '✅ 雷达跟随已启动（PID %d）。\n'
            '把腿伸到雷达前方 40cm 内即可锁定目标。\n'
            '保持约 50cm 距离：超过 65cm 会前进，小于 50cm 会后退。\n'
            '说"停止雷达跟随"可关闭。\n'
            '日志: %s' % (proc.pid, _LOG_FILE))

    except Exception as e:                          # noqa: BLE001
        import traceback
        _write_log('异常: %s\n%s' % (e, traceback.format_exc()))
        return '❌ 启动异常: %s\n详情见 %s' % (e, _LOG_FILE)


@tool
def stop_lidar_follow() -> str:
    """关闭雷达跟随，让小车停下来。

    当用户说"停止雷达跟随"、"别跟着我了"、"关闭跟随"时使用。
    """
    if not is_lidar_follow_running():
        return 'ℹ️ 雷达跟随未在运行'

    _write_log('请求停止')
    if stop_lidar_follow_process(force=False):
        if not is_lidar_follow_running():
            return '✅ 雷达跟随已停止，小车已停车'
        stop_lidar_follow_process(force=True)
        return '✅ 雷达跟随已强制停止'
    return '❌ 停止失败，请检查是否有残留进程'


@tool
def get_lidar_follow_status() -> str:
    """查询雷达跟随的运行状态。

    当用户问"雷达跟随开了吗"、"跟随状态"时使用。
    """
    if not is_lidar_follow_running():
        return '🔴 雷达跟随未运行'

    # 尝试读节点状态话题更准确，但工具在非 ROS 环境里跑，
    # 这里退化为「进程在 + 锁文件在」的简单判断
    lines = ['🟢 雷达跟随正在运行', '   前方 ±30° 内最近的障碍物就是跟随目标',
             '   保持约 50cm；超过 65cm 前进，小于 50cm 后退',
             '   目标丢失时立即停车等待']
    return '\n'.join(lines)
