import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 参数文件路径
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    ekf_config_path = os.path.join(fishbot_navigation2_dir, 'config', 'ekf.yaml')
    ekf_config = LaunchConfiguration('ekf_config', default=ekf_config_path)
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'ekf_config',
            default_value=ekf_config,
            description='EKF配置文件路径'
        ),

        # 协方差注入节点 —— 给 ESP32 /odom 补上合理协方差再转发给 EKF
        Node(
            package='fishbot_bringup',
            executable='odom_covariance_injector.py',
            output='screen'
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': False}],
            # 注意：不要添加 remapping ('odometry/filtered' -> 'odom')，
            # 因为 ekf.yaml 中 odom0: /odom_ekf，会导致反馈环路！
        ),
    ])