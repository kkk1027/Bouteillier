#!/usr/bin/env python3
"""不依赖 setuptools 入口脚本的语音节点启动器。

为什么需要这个文件
------------------
colcon --symlink-install 给 ament_python 包生成的 console_scripts 入口
是 **EASY-INSTALL 类型**，内容大意是：

    from importlib.metadata import distribution
    ... distribution('fishbot-voice').entry_points ...

它要求 Python 能通过 importlib.metadata 找到 `fishbot-voice.egg-info`。
而这个查找依赖 sys.path 里同时存在 build/fishbot_voice（放置 egg-info）
与 install/.../site-packages（放置 .egg-link）。一旦两者之一不同步
（例如只重建了一部分、或运行环境与 shell 的 PYTHONPATH 不一致），
就会抛：

    importlib.metadata.PackageNotFoundError:
        No package metadata was found for fishbot-voice

节点 0.1 秒就退出。本项目反复踩这个坑：单独在终端里 `python3 -c
"distribution('fishbot-voice')"` 一切正常，但 `ros2 launch` 拉起的子进程
却报 PackageNotFoundError —— 因为两者的 Python 与 sys.path 并不完全一致，
靠「预先检查」也无法准确预测。

本启动器的做法：**只做纯 Python 导入**
    from fishbot_voice.voice_control_node import main
完全绕开 importlib.metadata / pkg_resources 这一整套元数据机制。
只要 install 环境正常（import 能用），它就一定能跑。

保留原来的 console_scripts 入口只是为了兼容 `ros2 run fishbot_voice
voice_control_node`，launch 里一律用本脚本。
"""

import sys


def main() -> int:
    try:
        from fishbot_voice.voice_control_node import main as node_main
    except Exception as exc:                     # noqa: BLE001
        # 把 sys.path 一并打印出来。本项目排查过一整轮才发现：在终端里
        # `python3 -c "import fishbot_voice"` 正常，但 launch 拉起的子进程
        # 却 ModuleNotFoundError —— 两边 sys.path 并不一致。没有这行输出
        # 就只能靠猜。
        rel = [p for p in sys.path
               if 'fishbot_voice' in p or p in ('', '.')]
        sys.stderr.write(
            '\n❌ 无法导入 fishbot_voice.voice_control_node：%s\n'
            '   解释器：%s\n'
            '   相关 sys.path：\n%s\n'
            '   若上面没有 install/fishbot_voice/lib/python*/site-packages，'
            '说明环境未 source 正确。\n'
            '   若装了但包目录是空的/断链，重建该包：\n'
            '     rm -rf build/fishbot_voice install/fishbot_voice\n'
            '     colcon build --packages-select fishbot_voice '
            '--symlink-install\n'
            % (exc, sys.executable,
               '\n'.join('     - %s' % p for p in rel) or '     （无）'))
        return 1

    node_main()
    return 0


if __name__ == '__main__':
    sys.exit(main())
