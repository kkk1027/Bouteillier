#!/usr/bin/env python3
"""voice_control.launch.py

启动语音控制节点。

用法：
    ros2 launch fishbot_voice voice_control.launch.py
    ros2 launch fishbot_voice voice_control.launch.py linear_speed:=0.15
"""

import os
import subprocess
import sys

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _entry_point_problem() -> str:
    """检查语音节点能否真正启动。正常返回空字符串。

    背景（实际踩过的坑）：colcon --symlink-install 给 ament_python 包生成的
    console_scripts 入口是 EASY-INSTALL 类型，内部调
    importlib.metadata.distribution('fishbot-voice')，依赖 build 目录里的
    egg-info 同时出现在 sys.path 上。一旦不一致就抛
    PackageNotFoundError，节点 0.1 秒退出。

    更阴的是：**预先用同样的 python 去测 distribution() 却是成功的**，
    因为 launch 进程与它拉起的子进程 sys.path 并不完全一致 —— 所以靠
    元数据探测根本预测不准（本项目自检因此误判过，节点依然无限重启 44 次）。

    所以现在两件事一起做：
      1. 改用 scripts/run_voice_control_node.py（纯 import，不碰元数据）；
      2. 自检也改成**真正跑一次 import**，与运行时路径一致。
    """
    try:
        exe = os.path.join(get_package_prefix('fishbot_voice'),
                           'lib', 'fishbot_voice',
                           'run_voice_control_node.py')
        if not os.path.isfile(exe):
            return ('启动器未安装: %s\n'
                    '   需要重新编译该包才会生成。' % exe)
        if not os.access(exe, os.X_OK):
            return '启动器没有可执行权限: %s' % exe

        # 用子进程真实验证 import 链，与节点运行时一致
        res = subprocess.run(
            [sys.executable, '-c',
             'from fishbot_voice.voice_control_node import main'],
            capture_output=True, text=True, timeout=20)
        if res.returncode != 0:
            tail = (res.stderr or '').strip().splitlines()
            return ('无法导入 fishbot_voice.voice_control_node：%s'
                    % (tail[-1] if tail else '未知错误'))
        return ''
    except Exception as exc:          # noqa: BLE001 - 自检失败不该阻断启动
        return '自检异常: %s' % exc


def generate_launch_description():
    pkg_share = get_package_share_directory('fishbot_voice')
    default_yaml = os.path.join(pkg_share, 'config', 'voice_control.yaml')

    # 入口脚本坏了就别启动，否则 respawn 会无限刷屏
    problem = _entry_point_problem()
    if problem:
        return LaunchDescription([
            LogInfo(msg=(
                '\u274c 语音节点无法启动：%s\n'
                '   修复方法（在 fishbot_ws 目录下执行）：\n'
                '     rm -rf build/fishbot_voice install/fishbot_voice\n'
                '     colcon build --packages-select fishbot_voice '
                '--symlink-install\n'
                '   注意：只重启 launch 无效，必须重建该包。\n'
                '   其余功能（底盘、雷达、导航）不受影响。' % problem)),
        ])

    params_file_arg = DeclareLaunchArgument(
        # ★ 参数名必须带包名前缀。
        # bringup.launch.py 会把多个包的 launch 拼在一起，而 launch 的
        # 参数是全局作用域的。本项目实际踩过：语音和雷达都声明了
        # 叫 params_file 的参数，语音先生效，雷达的默认值被静默忽略，
        # 结果雷达拿着语音的 yaml 启动、port 回落 /dev/ydlidar 而秒退。
        'voice_params_file',
        default_value=default_yaml,
        description='语音控制参数文件路径',
    )
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed', default_value='0.2',
        description='直行线速度 m/s')
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed', default_value='1.0',
        description='原地转向角速度 rad/s')
    duration_arg = DeclareLaunchArgument(
        'action_duration', default_value='5.0',
        description='单个动作持续时间 s')
    # ★ 安全开关：默认 false = 干跑模式，绝不向 /cmd_vel 发布数据
    enable_motion_arg = DeclareLaunchArgument(
        'enable_motion', default_value='false',
        description='安全开关。false=干跑只打日志（默认）；true=实车下发速度')

    node = Node(
        package='fishbot_voice',
        # ★ 用 scripts/ 里的启动器，不用 console_scripts 入口。
        #   后者是 EASY-INSTALL 类型，依赖 importlib.metadata，在
        #   symlink-install 下会 PackageNotFoundError，节点秒退。
        executable='run_voice_control_node.py',
        name='voice_control_node',
        output='screen',
        emulate_tty=True,
        # ★★ 极其关键：不能只写 env={'PYTHONUNBUFFERED': '1'}！
        #
        # launch 的 ExecuteProcess（Node 底层用的就是它）语义是：
        #   env 为 None  → 完整继承父进程环境
        #   env 不为 None → **整体替换**子进程环境，不做合并
        #
        # 所以原来那句 `env={'PYTHONUNBUFFERED': '1'}` 把子进程环境清空成
        # 只剩这一个变量，PYTHONPATH / LD_LIBRARY_PATH / ROS_DOMAIN_ID 全没了。
        # 直接后果：`import fishbot_voice` 在子进程里 ModuleNotFoundError，
        # 节点 0.1 秒退出；即使侥幸 import 成功，也找不到 rclpy 的 C 扩展，
        # 而且会跑在默认 ROS 域而不是预期域。
        #
        # 本项目为此排查了很久（终端里一切正常、launch 里必崩，因为终端有
        # PYTHONPATH 而子进程没有）。正确做法是在父进程环境基础上追加。
        env=dict(os.environ, PYTHONUNBUFFERED='1'),
        parameters=[
            LaunchConfiguration('voice_params_file'),
            {
                # LaunchConfiguration 求值后是字符串，而节点里这几个参数
                # 声明为 double。必须用 ParameterValue 显式声明 value_type，
                # 否则 rclpy 在 declare_parameter 时会抛
                # InvalidParameterTypeException（字符串不能赋给 double 参数）。
                'linear_speed': ParameterValue(
                    LaunchConfiguration('linear_speed'), value_type=float),
                'angular_speed': ParameterValue(
                    LaunchConfiguration('angular_speed'), value_type=float),
                'action_duration': ParameterValue(
                    LaunchConfiguration('action_duration'), value_type=float),
                # 安全开关：必须显式声明 bool 类型，否则字符串 "false"
                # 赋给 bool 参数会报类型错误
                'enable_motion': ParameterValue(
                    LaunchConfiguration('enable_motion'), value_type=bool),
            },
        ],
        # ★ 必须开 respawn。
        # 语音节点一旦因任何原因退出（依赖包元数据损坏、串口被占、
        # 麦克风设备不在……），没有 respawn 就永久失联，而 bringup 日志早
        # 就被雷达输出刷满，人根本注意不到它已经死了。
        # 本项目实际踩过：EASY-INSTALL 入口脚本因 egg-info 损坏而报
        # PackageNotFoundError，节点挂掉后一直没人拉起，表现为「语音没反应」。
        # respawn 加上后，即使遇到瞬时故障也能自愈。
        respawn=True,
        respawn_delay=5.0,
    )

    return LaunchDescription([
        params_file_arg,
        linear_speed_arg,
        angular_speed_arg,
        duration_arg,
        enable_motion_arg,
        node,
    ])
