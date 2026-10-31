"""
mutual_exclusion.py
与导航、其他跟随功能的互斥。

为什么需要：
    本项目**没有 twist_mux**，所有节点直接往 /cmd_vel 发指令。
    跟随、导航、视觉看护、遥控都能发，两个一起跑会互相覆盖，
    车就原地抽搐。所以用锁文件做约定式互斥。

锁文件语义：
    跟随节点启动时写（内容是自己的 PID），退出时删。
    导航工具启动前检查：文件在 + 进程活着 → 拒绝导航。
    同时验证进程还在，避免 kill -9 后锁文件残留导致导航永远无法启动。
"""

import os
import subprocess

LOCK_FILE = '/tmp/fishbot_lidar_follow.lock'

# 用于判断跟随进程是否还活着的匹配串
_PROC_PATTERN = 'lidar_follow'


def write_lock(pid=None):
    """写锁。返回是否成功。"""
    try:
        with open(LOCK_FILE, 'w') as f:
            f.write(str(pid if pid is not None else os.getpid()))
        return True
    except Exception:                               # noqa: BLE001
        return False


def remove_lock():
    """删锁。"""
    try:
        if os.path.exists(LOCK_FILE):
            os.remove(LOCK_FILE)
    except Exception:                               # noqa: BLE001
        pass


def _follow_process_alive():
    """用 pgrep 判断跟随节点进程是否还在。

    ★ 必须排除自身 PID。
    `pgrep -f "lidar_follow"` 会匹配命令行里含该字符串的**任何**进程，
    包括「正在 import 本模块的 Python 进程」——
    如果调用方的命令行里恰好有这个字样（比如用户敲了
    `python3 -c "...lidar_follow_active()..."`），
    pgrep 就会把它自己算进去，于是「锁在但进程没了」被误判成
    「进程还活着」，陈旧锁永远清不掉，导航被永久阻塞。
    本项目实际踩过这个坑。

    pgrep 不可用时返回 True（宁可多拒一次，也不要撞车）。
    """
    try:
        out = subprocess.run(
            ['pgrep', '-f', _PROC_PATTERN],
            capture_output=True, text=True, timeout=3)
        me = os.getpid()
        for tok in out.stdout.split():
            try:
                pid = int(tok)
            except ValueError:
                continue
            if pid == me:
                continue
            return True
        return False
    except Exception:                               # noqa: BLE001
        return True


def follow_active():
    """跟随是否正在占用底盘。

    有锁 + 进程活着 → True
    有锁但进程没了 → 陈旧锁，顺手清掉，返回 False
    """
    if not os.path.exists(LOCK_FILE):
        return False

    if _follow_process_alive():
        return True

    # 陈旧锁：节点被 kill -9 了，锁没来得及删
    print('[互斥] 检测到陈旧的跟随锁，已清理')
    remove_lock()
    return False


def blocked_message():
    """跟随占用底盘时给用户的提示，否则返回 None。"""
    if follow_active():
        return ('⚠️ 雷达跟随正在运行，无法启动导航'
                '（两者都会控制 /cmd_vel，同时跑会互相覆盖）。\n'
                '请先说「停止雷达跟随」，或在网页上关闭跟随，然后再导航。')
    return None
