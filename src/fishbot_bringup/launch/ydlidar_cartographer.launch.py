import os
from launch import LaunchDescription, conditions
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    ydlidar_pkg = FindPackageShare('ydlidar')
    fishbot_bringup_dir = get_package_share_directory('fishbot_bringup')
    fishbot_description_pkg = FindPackageShare('fishbot_description')
    
    # Cartographer配置目录（默认：fishbot_bringup 包内的 config/）
    default_config_dir = os.path.join(fishbot_bringup_dir, 'config')
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir', default=default_config_dir)
    config_basename = LaunchConfiguration(
        'configuration_basename', default='fishbot_cartographer.lua')
    
    # 激光雷达节点（LifecycleNode）
    ydlidar_node = LifecycleNode(
        package='ydlidar',
        executable='ydlidar_node',
        name='ydlidar_node',
        output='screen',
        parameters=[os.path.join(ydlidar_pkg, 'params', 'ydlidar.yaml')],
        namespace='/',
    )
    
    # 配置激光雷达生命周期节点
    configure_ydlidar = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/ydlidar_node', 'configure'],
        output='screen'
    )
    
    activate_ydlidar = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/ydlidar_node', 'activate'],
        output='screen'
    )
    
    # 里程计TF广播节点
    odom2tf_node = Node(
        package='fishbot_bringup',
        executable='odom2tf',
        name='odom2tf',
        output='screen'
    )
    
    # URDF机器人状态发布节点
    fishbot_description_dir = get_package_share_directory('fishbot_description')
    urdf_path = os.path.join(fishbot_description_dir, 'urdf', 'fishbot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content
        }],
        output='screen'
    )
    
    # ==================== 声明参数 ====================
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
        'resolution', default_value='0.05',
        description='Resolution of a grid cell in the published occupancy grid'
    )
    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec', default_value='1.0',
        description='OccupancyGrid publishing period (seconds)'
    )
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start RViz2 automatically'
    )

    # ==================== Cartographer建图节点 ====================
    # 注意：scan_qos_bridge 已移除。ydlidar_node 现在使用 RELIABLE QoS
    # 直接发布 /scan，无需桥接。
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', config_basename
        ],
        output='screen'
    )
    
    # ==================== Cartographer地图发布节点 ====================
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-resolution', LaunchConfiguration('resolution'),
            '-publish_period_sec', LaunchConfiguration('publish_period_sec')
        ]
    )
    
    # ==================== RViz2可视化 ====================
    rviz_config_dir = os.path.join(
        fishbot_bringup_dir, 'config', 'cartographer_slam.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        condition=conditions.IfCondition(use_rviz),
        output='screen'
    )
    
    # ==================== LaunchDescription ====================
    return LaunchDescription([
        # 参数声明
        declare_cartographer_config_dir,
        declare_configuration_basename,
        declare_resolution,
        declare_publish_period_sec,
        declare_use_rviz,

        # ydlidar 激光雷达
        ydlidar_node,
        configure_ydlidar,
        activate_ydlidar,

        # 机器人 TF
        odom2tf_node,
        robot_state_publisher_node,

        # Cartographer SLAM
        cartographer_node,
        cartographer_occupancy_grid_node,

        # 可视化
        rviz2_node,
    ])