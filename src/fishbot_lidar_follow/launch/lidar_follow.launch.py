#!/usr/bin/env python3
"""
lidar_follow.launch.py
启动雷达跟随节点。

用法：
    ros2 launch fishbot_lidar_follow lidar_follow.launch.py
    ros2 launch fishbot_lidar_follow lidar_follow.launch.py \
        params_file:=/path/to/custom.yaml
    ros2 launch fishbot_lidar_follow lidar_follow.launch.py log_level:=debug

前置条件：
    * 雷达已启动（/scan 有数据）—— 通常 bringup 已包含
    * 底盘已启动（/cmd_vel 有人听）

本 launch 只负责跟随节点本身，不会去拉 bringup。

想先离线验证？另开两个终端：
    ros2 run fishbot_lidar_follow lidar_follow_sim
    ros2 launch fishbot_lidar_follow lidar_follow.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('fishbot_lidar_follow')
    default_params = os.path.join(pkg_dir, 'config', 'lidar_follow.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='雷达跟随参数文件路径',
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别 (debug/info/warn)',
    )

    follow_node = Node(
        package='fishbot_lidar_follow',
        executable='lidar_follow',
        name='lidar_follow_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        # 跟随节点退出不自动重启：它一旦崩了说明有问题，
        # 自动重启会掩盖故障，还可能带着错误状态继续发指令。
        respawn=False,
        emulate_tty=True,
    )

    return LaunchDescription([
        params_arg,
        log_level_arg,
        follow_node,
    ])
