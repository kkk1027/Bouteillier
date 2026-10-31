"""
paths.py
集中定位工作空间路径。

背景：原代码在多个文件里把路径写死成作者机器上的位置：
    FISHBOT_WS   默认 '/home/ke/chapt9_update/fishbot_ws'
    DETECTION_WS 默认 '/home/ke/detection_ws'
换到别的机器后，ros2 launch 直接失败，前端只看到「连接失败 (退出码: 1)」。

现在统一走这里：优先环境变量，其次由本文件位置推导，最后常见路径兜底。
"""

import os

# 本文件位于 <ws>/src/autopatrol_robot/autopatrol_robot/paths.py
# （--symlink-install 时从 build/ 下的副本运行，向上 3 级同样是 <ws>）
_HERE = os.path.dirname(os.path.abspath(__file__))
_DERIVED_WS = os.path.dirname(os.path.dirname(os.path.dirname(_HERE)))

# fishbot_ws 常见位置（兜底用）
_FISHBOT_WS_CANDIDATES = [
    os.path.expanduser('~/bishe_ws/bishe-main/fishbot_ws'),
    os.path.expanduser('~/fishbot_ws'),
    os.path.expanduser('~/chapt9_update/fishbot_ws'),
]


def fishbot_ws() -> str:
    """返回 fishbot_ws 工作空间根目录（绝对路径）。

    查找顺序：FISHBOT_WS 环境变量 → 由本文件位置推导 → 常见路径兜底。
    命中条件是该目录下存在 install/setup.bash。
    """
    candidates = []
    env_ws = os.environ.get('FISHBOT_WS', '').strip()
    if env_ws:
        candidates.append(os.path.abspath(os.path.expanduser(env_ws)))

    candidates.append(_DERIVED_WS)
    candidates.extend(_FISHBOT_WS_CANDIDATES)

    for ws in candidates:
        if ws and os.path.isfile(os.path.join(ws, 'install', 'setup.bash')):
            return ws

    # 都没装好时返回推导值：至少保证报错信息里路径是这台机器上的
    return _DERIVED_WS


def detection_ws() -> str:
    """返回检测（摄像头跟随）工作空间根目录。

    这是一个**可选**的独立工作空间（esp32_cam_detector），
    只影响「看护/跟随模式」，不影响建图导航。
    本机没有该目录时，跟随功能会提示路径不存在，其余功能正常。
    """
    env_ws = os.environ.get('DETECTION_WS', '').strip()
    if env_ws:
        return os.path.abspath(os.path.expanduser(env_ws))

    for cand in (
        os.path.expanduser('~/bishe_ws/bishe-main/detection_ws'),
        os.path.expanduser('~/detection_ws'),
        os.path.expanduser('~/bishe_ws/detection_ws'),
    ):
        if os.path.isdir(cand):
            return cand

    # 未找到时给一个同级的合理默认位置，便于报错信息定位
    return os.path.expanduser('~/bishe_ws/bishe-main/detection_ws')
