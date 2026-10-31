"""
FishBot Cartographer SLAM 建图启动文件

使用方式：
    # 启动建图（默认配置）
    ros2 launch fishbot_bringup cartographer_slam.launch.py

    # 启动建图（自定义参数）
    ros2 launch fishbot_bringup cartographer_slam.launch.py \
        configuration_basename:=fishbot_cartographer.lua \
        resolution:=0.05 \
        publish_period_sec:=1.0 \
        use_rviz:=true

依赖：
    - bringup.launch.py 需先启动（urdf2tf + odom2tf + micro_ros_agent + ydlidar）
    - Cartographer ROS2 包已安装 (ros-humble-cartographer-ros)
    - ydlidar_node 以 RELIABLE QoS 直接发布 /scan（无需 QoS 桥接）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # ==================== 包路径 ====================
    fishbot_bringup_dir = get_package_share_directory('fishbot_bringup')

    # ==================== Launch 参数 ====================
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # Cartographer 配置目录和文件名
    # 默认路径：包内的 config/ 目录
    default_config_dir = os.path.join(fishbot_bringup_dir, 'config')
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=default_config_dir
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='fishbot_cartographer.lua'
    )

    # 占用栅格地图参数
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # RViz2 配置
    rviz_config_dir = os.path.join(
        fishbot_bringup_dir, 'config', 'cartographer_slam.rviz'
    )

    # ==================== 声明参数 ====================
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 automatically'
    )

    declare_cartographer_config_dir = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=default_config_dir,
        description='Full path to Cartographer config directory'
    )

    declare_configuration_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value='fishbot_cartographer.lua',
        description='Name of the Cartographer lua configuration file'
    )

    declare_resolution = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of a grid cell in the published occupancy grid'
    )

    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='OccupancyGrid publishing period (seconds)'
    )

    # ==================== 节点定义 ====================

    # --- Cartographer SLAM 主节点 ---
    # 输入：/scan (ydlidar 直接以 RELIABLE QoS 发布) + /odom (里程计)
    # 注意：scan_qos_bridge 已移除，因为 ydlidar_node 现在使用 RELIABLE QoS
    # 输出：map→odom TF + 子图数据
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
        ]
    )

    # --- Cartographer 占用栅格地图发布节点 ---
    # 将 Cartographer 的子图数据转换为 ROS 标准的 /map 话题
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec
        ],
        output='screen'
    )

    # --- RViz2 可视化（可选） ---
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    # ==================== LaunchDescription ====================
    return LaunchDescription([
        # 参数声明
        declare_use_sim_time,
        declare_use_rviz,
        declare_cartographer_config_dir,
        declare_configuration_basename,
        declare_resolution,
        declare_publish_period_sec,

        # 核心节点
        cartographer_node,
        cartographer_occupancy_grid_node,

        # 可视化
        rviz2_node,
    ])
