#!/usr/bin/env python3
"""
lidar_follow_sim.launch.py
一键启动「仿真雷达 + 跟随节点」，纯离线，不碰真实底盘。

用法：
    ros2 launch fishbot_lidar_follow lidar_follow_sim.launch.py
    ros2 launch fishbot_lidar_follow lidar_follow_sim.launch.py \
        scenario:=walk_close

这个 launch 会启动两个节点：
    lidar_follow_sim    假雷达（发 /scan）+ 打印 /cmd_vel
    lidar_follow_node   真正的跟随节点

注意：仿真节点默认只打印速度，不转发到真实 /cmd_vel。
所以即使底盘连着，车也不会动。
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

    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='walk_away',
        description='仿真场景：walk_away / walk_close / sideways / '
                    'dropout / static',
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='跟随参数文件路径',
    )

    # 注意参数顺序：仿真节点和跟随节点用不同的 scan/cmdVel 话题会撕不起来，
    # 这里统一用默认的 /scan 和 /cmd_vel。
    sim_node = Node(
        package='fishbot_lidar_follow',
        executable='lidar_follow_sim',
        name='lidar_follow_sim',
        output='screen',
        parameters=[{'scenario': LaunchConfiguration('scenario')}],
        emulate_tty=True,
    )

    follow_node = Node(
        package='fishbot_lidar_follow',
        executable='lidar_follow',
        name='lidar_follow_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True,
    )

    return LaunchDescription([
        scenario_arg,
        params_arg,
        # 仿真先起，保证跟随节点启动时已经有数据
        sim_node,
        follow_node,
    ])
