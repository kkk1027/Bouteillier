#!/usr/bin/env python3
"""引导页 UI 联调用的独立测试实例（端口 5051）。

为什么要单独起一个：

  * 主服务（5000 端口）当前正连着实物机器人 —— `is_connected=true`，
    bringup / micro_ros_agent / ydlidar 全在跑。
  * 重启主服务会让 bringup 变成孤儿进程，而 micro_ros_agent 还活着，
    ESP32 一有 /cmd_vel 就会驱动底盘 —— 这是必须要避免的状态。
  * 本脚本只 `import web_controller` 后起一个 Flask 服务：
    - `main()` 不执行 → 不会 rclpy.init()、不会创建 CarController
    - 因此**没有任何 /cmd_vel 发布者**，纯 UI/接口联调
    - 但模板、静态文件、/guide/* 路由全都和主服务一致

用法：
    cd src/autopatrol_robot/autopatrol_robot
    python3 scripts/guide_dev_server.py [端口]

浏览器打开 http://127.0.0.1:5051/?guide=1
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(HERE)
sys.path.insert(0, PKG_DIR)

# 关闭代理，否则 127.0.0.1 的请求可能被系统代理接管
os.environ.setdefault('NO_PROXY', '127.0.0.1,localhost,::1')
os.environ.setdefault('no_proxy', os.environ['NO_PROXY'])

import web_controller as W                                # noqa: E402


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 5051

    assert W.car_controller is None, \
        'car_controller 不为 None，说明这个实例可能持有 /cmd_vel 发布者，拒绝启动'

    print('=' * 60)
    print('  引导页联调实例（只读 UI，无 ROS 节点、无 /cmd_vel）')
    print()
    print('  已验证能跑的画面：')
    print('    http://127.0.0.1:%d/?guide=2   绘制调试（免连接，推荐）' % port)
    print('    http://127.0.0.1:%d/?guide=1   强制进入引导第 1 步' % port)
    print('    http://127.0.0.1:%d/?guide=0   跳过引导' % port)
    print()
    print('  绘制结果导出到：maps/guide_render/')
    print('    <map>_render_vector.png 纯线稿（主产物：标签 + 线条，无栅格）')
    print('    <map>_render.png        带标注的三态图（答辩 / 文档）')
    print('    <map>_render_clean.png  纯三态底图（备查）')
    print('  Ctrl+C 退出')
    print('=' * 60)

    # 用 socketio.run 才能让 socket.io 长连接也能工作
    W.socketio.run(
        W.app,
        host='0.0.0.0',
        port=port,
        debug=False,
        use_reloader=False,
        allow_unsafe_werkzeug=True,
    )


if __name__ == '__main__':
    main()
