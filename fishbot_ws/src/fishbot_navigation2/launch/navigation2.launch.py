import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取与拼接默认路径
    fishbot_navigation2_dir = get_package_share_directory(
        'fishbot_navigation2')
    fishbot_bringup_dir = get_package_share_directory(
        'fishbot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(fishbot_navigation2_dir, 'maps', 'fishbot_map1.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))
    ekf_config_path = os.path.join(fishbot_navigation2_dir, 'config', 'ekf.yaml')
    
    # URDF → TF（发布 base_footprint→base_link→laser_link 静态变换）
    # 必须最先启动，否则 costmap/AMCL 找不到 robot_base_frame
    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [fishbot_bringup_dir, '/launch', '/urdf2tf.launch.py']),
    )
    
    # odom→base_footprint TF广播
    odom2tf = Node(
        package='fishbot_bringup',
        executable='odom2tf',
        output='screen'
    )
    
    # EKF节点（用于融合里程计和IMU）
    # 注意：不要添加 remapping ('odometry/filtered' -> 'odom')，
    # 因为 ekf.yaml 中 odom0: /odom_ekf，EKF 会同时读写 /odom 导致反馈环路！
    # 正确做法：odom2tf 节点直接订阅 EKF 的输出话题 /odometry/filtered
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[{'use_sim_time': False}, ekf_config_path]
    )
    
    # 协方差注入节点 —— 给 ESP32 /odom 补上合理协方差再转发给 EKF
    odom_cov_injector = Node(
        package='fishbot_bringup',
        executable='odom_covariance_injector.py',
        output='screen'
    )
    
    # IMU陀螺偏置校准节点（修正MPU6050的wz漂移）
    imu_bias_remover = Node(
        package='fishbot_bringup',
        executable='imu_bias_remover',
        output='screen'
    )

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        # 1️⃣ URDF→TF（发布 base_footprint→base_link→laser_link，costmap/AMCL依赖）
        urdf2tf,

        # 2️⃣ odom→base_footprint TF广播
        odom2tf,

        # IMU偏置校准必须在EKF之前启动
        imu_bias_remover,

        # 协方差注入必须在EKF之前启动（/odom → /odom_ekf）
        odom_cov_injector,

        # EKF里程计和IMU融合节点
        ekf_node,

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
