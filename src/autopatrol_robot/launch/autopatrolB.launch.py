"""
autopatrolB.launch.py — 方案B launch 文件

直接复制自 autopatrol.launch.py，仅修改 executable 为 patrol_nodeB。
"""

import os
import yaml
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'target_indices',
            default_value='1,2,3',
            description='Comma-separated list of waypoint indices (1-based)'
        ),
        DeclareLaunchArgument(
            'target_coordinates',
            default_value='',
            description='Direct coordinates x,y,yaw to navigate to (overrides target_indices)'
        ),
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    autopatrol_robot_dir = get_package_share_directory('autopatrol_robot')

    patrol_config_path = os.path.join(autopatrol_robot_dir, 'config', 'patrol_config.yaml')
    with open(patrol_config_path, 'r') as f:
        config = yaml.safe_load(f)

    coords_str = LaunchConfiguration('target_coordinates').perform(context)
    if coords_str:
        parts = [float(p.strip()) for p in coords_str.split(',')]
        if len(parts) not in (2, 3):
            raise ValueError(f'target_coordinates 格式错误: {coords_str}，应为 x,y,yaw')
        if len(parts) == 2:
            parts.append(0.0)
        selected_points = parts
        print(f"直接坐标导航: x={parts[0]}, y={parts[1]}, yaw={parts[2]}")
    else:
        indices_str = LaunchConfiguration('target_indices').perform(context)
        indices = [int(i.strip()) for i in indices_str.split(',')]

        all_targets = config['patrol_node']['ros__parameters']['target_points']

        waypoints = []
        for i in range(0, len(all_targets), 3):
            waypoints.append(all_targets[i:i+3])

        selected_points = []
        for idx in indices:
            if 1 <= idx <= len(waypoints):
                selected_points.extend(waypoints[idx-1])
                print(f"选择航点 {idx}: {waypoints[idx-1]}")
            else:
                print(f"警告: 航点索引 {idx} 超出范围 (1-{len(waypoints)})")

        if not selected_points:
            print(f"警告: 没有有效选择，使用默认航点1")
            selected_points = waypoints[0]

    initial_point = config['patrol_node']['ros__parameters'].get('initial_point', [0.0, 0.0, 0.0])

    action_node_patrol = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='patrol_nodeB',           # ← 方案B
        parameters=[
            {'initial_point': initial_point},
            {'target_points': selected_points},
            {'image_save_path': ''}
        ]
    )

    return [action_node_patrol]
