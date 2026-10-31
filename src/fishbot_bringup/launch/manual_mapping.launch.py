"""FishBot 手动建图一键启动（Cartographer 2D + IMU 融合）。

用法：
    # 一键启动：底盘 bringup + IMU/EKF + Cartographer + 存图服务 + RViz
    ros2 launch fishbot_bringup manual_mapping.launch.py

    # 底盘已经由网页/其他 launch 启动，只起建图链路
    ros2 launch fishbot_bringup manual_mapping.launch.py include_bringup:=false

    # 已有 EKF/imu_bias_remover 在跑（例如 navigation2.launch.py），避免重复启动
    ros2 launch fishbot_bringup manual_mapping.launch.py \
        include_bringup:=false start_ekf:=false

    # 不起 RViz（远程/无显示器）
    ros2 launch fishbot_bringup manual_mapping.launch.py use_rviz:=false

建图结束后存图：
    ros2 service call /manual_mapping/save_map \
        custom_interfaces/srv/SaveMap \
        "{name: 'room1', directory: '', overwrite: false, save_pbstream: true}"

    # 丢弃本次建图（不落盘）
    ros2 service call /manual_mapping/discard_map std_srvs/srv/Trigger "{}"

    # 查询状态（message 为 JSON）
    ros2 service call /manual_mapping/get_status std_srvs/srv/Trigger "{}"

关键约束：
  - Cartographer 与 AMCL/Nav2 不能同时发布 map->odom，建图前请关闭导航。
  - 本 launch 不启动任何遥控节点，也不会自动发送 /cmd_vel。
  - IMU 融合依赖 base_link -> imu_link 静态 TF（默认由本 launch 发布）。
  - 启动后保持静止 3~5 s，让 imu_bias_remover 完成陀螺零偏标定。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('fishbot_bringup')
    navigation_dir = get_package_share_directory('fishbot_navigation2')

    config_dir = os.path.join(bringup_dir, 'config')
    ekf_config = os.path.join(navigation_dir, 'config', 'ekf.yaml')
    rviz_config = os.path.join(bringup_dir, 'config', 'cartographer_slam.rviz')

    # ---------------- Launch 参数 ----------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    include_bringup = LaunchConfiguration('include_bringup')
    start_ekf = LaunchConfiguration('start_ekf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_arg = LaunchConfiguration('rviz_config')
    config_basename = LaunchConfiguration('configuration_basename')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    imu_x = LaunchConfiguration('imu_x')
    imu_y = LaunchConfiguration('imu_y')
    imu_z = LaunchConfiguration('imu_z')
    imu_roll = LaunchConfiguration('imu_roll')
    imu_pitch = LaunchConfiguration('imu_pitch')
    imu_yaw = LaunchConfiguration('imu_yaw')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='使用仿真时钟（实车为 false）')

    declare_include_bringup = DeclareLaunchArgument(
        'include_bringup', default_value='true',
        description='是否一并启动底盘 bringup（微ROS/串口/雷达/TF/语音已强制关闭）')

    # 默认跟随 include_bringup：一键启动时启动 EKF 与 IMU 偏置节点；
    # 若外部已经跑了 navigation2.launch.py（内含 EKF），需显式设为 false。
    declare_start_ekf = DeclareLaunchArgument(
        'start_ekf', default_value=LaunchConfiguration('include_bringup'),
        description='是否启动 imu_bias_remover + EKF；外部已有 EKF 时设为 false')

    declare_publish_imu_tf = DeclareLaunchArgument(
        'publish_imu_tf', default_value='true',
        description='是否发布 base_link -> imu_link 静态 TF（IMU 建图必需）')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='是否启动 RViz2')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config', default_value=rviz_config,
        description='RViz2 配置文件路径')

    declare_config_dir = DeclareLaunchArgument(
        'cartographer_config_dir', default_value=config_dir,
        description='Cartographer lua 配置目录')

    declare_config_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value='fishbot_cartographer_manual.lua',
        description='Cartographer lua 配置文件名（IMU 融合版）')

    declare_resolution = DeclareLaunchArgument(
        'resolution', default_value='0.05',
        description='占用栅格分辨率 (m/cell)')

    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec', default_value='1.0',
        description='占用栅格发布周期 (s)')

    declare_imu_x = DeclareLaunchArgument(
        'imu_x', default_value='0.0', description='IMU 相对 base_link 的 x (m)')
    declare_imu_y = DeclareLaunchArgument(
        'imu_y', default_value='0.0', description='IMU 相对 base_link 的 y (m)')
    declare_imu_z = DeclareLaunchArgument(
        'imu_z', default_value='0.0', description='IMU 相对 base_link 的 z (m)')
    declare_imu_roll = DeclareLaunchArgument(
        'imu_roll', default_value='0.0', description='IMU 安装 roll (rad)')
    declare_imu_pitch = DeclareLaunchArgument(
        'imu_pitch', default_value='0.0', description='IMU 安装 pitch (rad)')
    declare_imu_yaw = DeclareLaunchArgument(
        'imu_yaw', default_value='0.0', description='IMU 安装 yaw (rad)')

    declare_map_topic = DeclareLaunchArgument(
        'map_topic', default_value='/map',
        description='Cartographer 占用栅格话题')

    declare_map_dir = DeclareLaunchArgument(
        'map_dir', default_value='',
        description='地图保存目录；空则使用 <workspace>/src/fishbot_navigation2/maps')

    declare_map_name_prefix = DeclareLaunchArgument(
        'map_name_prefix', default_value='manual_',
        description='未指定地图名时使用的前缀，后接 YYYYmmdd_HHMMSS')

    declare_final_map_timeout = DeclareLaunchArgument(
        'final_map_timeout_sec', default_value='20.0',
        description='finish_trajectory 后等待最终 /map 的超时时间')

    declare_final_map_settle = DeclareLaunchArgument(
        'final_map_settle_sec', default_value='3.0',
        description='收到最终 /map 后额外等待的稳定时间')

    declare_map_saver_timeout = DeclareLaunchArgument(
        'map_saver_timeout_sec', default_value='30.0',
        description='map_saver_cli 超时时间')

    declare_save_pbstream = DeclareLaunchArgument(
        'save_pbstream', default_value='true',
        description='存图时是否同时保存 Cartographer .pbstream')

    declare_overwrite = DeclareLaunchArgument(
        'overwrite', default_value='false',
        description='同名地图文件是否允许覆盖')

    declare_preflight_check = DeclareLaunchArgument(
        'preflight_check', default_value='true',
        description='启动后做一次只读自检（传感器/TF 告警，不发任何运动指令）')

    declare_preflight_delay = DeclareLaunchArgument(
        'preflight_delay_sec', default_value='20.0',
        description='启动后多少秒执行自检')

    # ---------------- 底盘 bringup ----------------
    # 建图期间强制关闭语音，避免"喊一句话车就动"；语音遥控与手动建图不能混用。
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'enable_voice': 'false',
            'enable_voice_motion': 'false',
        }.items(),
        condition=IfCondition(include_bringup),
    )

    # ---------------- IMU 偏置 + EKF ----------------
    # odom2tf 订阅的是 /odometry/filtered，没有 EKF 就不会有 odom->base_footprint TF。
    imu_bias_remover = Node(
        package='fishbot_bringup',
        executable='imu_bias_remover',
        name='imu_bias_remover',
        output='screen',
        condition=IfCondition(start_ekf),
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_ekf),
    )

    # ---------------- IMU 静态 TF ----------------
    # FishBot URDF 没有 imu_link；Cartographer 在 tracking_frame=imu_link 时
    # 必须能查到 imu_link。默认把 IMU 与 base_link 视为重合，安装偏移可调。
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_link_static_tf',
        output='screen',
        arguments=[
            '--x', imu_x, '--y', imu_y, '--z', imu_z,
            '--roll', imu_roll, '--pitch', imu_pitch, '--yaw', imu_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ],
        condition=IfCondition(publish_imu_tf),
    )

    # ---------------- Cartographer ----------------
    # 传感器口径与实机 stemm 包一致：原始 /odom + 去偏置 /imu_corrected + /scan；
    # EKF 只负责 odom->base_footprint TF，不参与 Cartographer 输入。
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', config_basename,
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('imu', '/imu_corrected'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec,
        ],
    )

    # ---------------- 手动建图管理节点 ----------------
    manager = Node(
        package='fishbot_bringup',
        executable='manual_mapping_manager.py',
        name='manual_mapping_manager',
        output='screen',
        parameters=[{
            'map_topic': LaunchConfiguration('map_topic'),
            'map_dir': LaunchConfiguration('map_dir'),
            'map_name_prefix': LaunchConfiguration('map_name_prefix'),
            'trajectory_id': 0,
            'final_map_timeout_sec': ParameterValue(
                LaunchConfiguration('final_map_timeout_sec'),
                value_type=float),
            'final_map_settle_sec': ParameterValue(
                LaunchConfiguration('final_map_settle_sec'),
                value_type=float),
            'map_saver_timeout_sec': ParameterValue(
                LaunchConfiguration('map_saver_timeout_sec'),
                value_type=float),
            'save_pbstream': ParameterValue(
                LaunchConfiguration('save_pbstream'), value_type=bool),
            'overwrite': ParameterValue(
                LaunchConfiguration('overwrite'), value_type=bool),
            'preflight_check': ParameterValue(
                LaunchConfiguration('preflight_check'), value_type=bool),
            'preflight_delay_sec': ParameterValue(
                LaunchConfiguration('preflight_delay_sec'), value_type=float),
        }],
    )

    # ---------------- RViz ----------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_arg],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_include_bringup,
        declare_start_ekf,
        declare_publish_imu_tf,
        declare_use_rviz,
        declare_rviz_config,
        declare_config_dir,
        declare_config_basename,
        declare_resolution,
        declare_publish_period_sec,
        declare_imu_x,
        declare_imu_y,
        declare_imu_z,
        declare_imu_roll,
        declare_imu_pitch,
        declare_imu_yaw,
        declare_map_topic,
        declare_map_dir,
        declare_map_name_prefix,
        declare_final_map_timeout,
        declare_final_map_settle,
        declare_map_saver_timeout,
        declare_save_pbstream,
        declare_overwrite,
        declare_preflight_check,
        declare_preflight_delay,

        LogInfo(msg='🗺️  手动建图模式：不会自动发送 /cmd_vel，请用 Web/Qt/键盘遥控'),
        LogInfo(msg='⚠️  建图前请确认 AMCL/Nav2 未运行；启动后保持静止 3~5 s 让 IMU 完成零偏标定'),

        bringup,
        imu_bias_remover,
        ekf_node,
        imu_tf,
        cartographer_node,
        occupancy_grid_node,
        manager,
        rviz_node,
    ])
