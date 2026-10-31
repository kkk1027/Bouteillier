#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar')
    # ★ 参数名必须用 ydlidar 专属前缀，不能叫 params_file。
    #
    # 真实踩坑（2026-10-30）：bringup.launch.py 里同时 include 了
    # voice_control.launch.py 和本文件，两者都声明了名叫 params_file 的
    # launch argument。launch 的配置是全局作用域的：语音模块先 include，
    # 把 params_file 设成了 voice_control.yaml；5 秒后 TimerAction 执行
    # 到本文件时，DeclareLaunchArgument 发现同名键已有值，静默忽略默认值。
    # 结果 ydlidar_node 拿着 voice_control.yaml 启动 —— 那份文件里没有
    # ydlidar_node 段，所有参数回落默认值（port=/dev/ydlidar、
    # baudrate=230400），0.5ms 内失败退出，respawn 无限重试 1000+ 次，
    # /scan 永久消失。而单独 ros2 launch ydlidar 却完全正常，极具迷惑性。
    parameter_file = LaunchConfiguration('ydlidar_params_file')
    node_name = 'ydlidar_node'

    params_declare = DeclareLaunchArgument(
        'ydlidar_params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar.yaml'),
        description='ydlidar 驱动的参数文件路径（独立名字，避免与其他包冲突）')

    # 说明（2026-10-30）：
    #   ydlidar_node 实际是普通 Node（源码里没有生命周期回调，也没有
    #   on_configure/on_activate），原来用 LifecycleNode 启动只是名义上的，
    #   configure/activate 服务并不存在。这里改用普通 Node，并打开 respawn：
    #   串口桥接（ros_serial2wifi tcp_server）晚于本节点接入时，节点会因
    #   拿不到雷达数据而退出，respawn 会自动重试直到桥接就绪。
    #   注意：ydlidar_cartographer.launch.py 用的是它自己的 LifecycleNode，
    #   不受此处影响。
    driver_node = Node(package='ydlidar',
                       executable='ydlidar_node',
                       name='ydlidar_node',
                       output='screen',
                       emulate_tty=True,
                       respawn=True,
                       respawn_delay=5.0,
                       parameters=[parameter_file],
                       namespace='/',
                       )
    return LaunchDescription([
        params_declare,
        driver_node,
    ])
