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

    # ==========================================================
    # 避免与 bringup 重复启动辅助节点
    # ----------------------------------------------------------
    # 实际用法是「先 bringup 再 navigation」，而 bringup 已经启动了
    # urdf2tf / odom2tf / odom_covariance_injector /
    # imu_bias_remover / ekf_node / imu_link 静态 TF。
    # 两个 launch 都起就会变成两份进程：
    #   - 静态 TF 被两个进程重复发布 → TF_REPEATED_DATA 刷屏
    #   - /odom_ekf 被两个注入节点重复发布 → EKF 吃到重复输入
    #   - 两个同名 ekf_filter_node → 节点名冲突，或两个滤波器
    #     交错往 /odometry/filtered 写 → odom→base_link 反复跳变
    # 所以默认 false，认为 bringup 已提供；单独跑导航调试时再置 true。
    #
    # ★ 历史坑（2026-10-31）：上面的理由曾经**不成立** ——
    #   本文件默认 false，而当时的 bringup 里根本没有 ekf_node，
    #   两边都以为对方起了 EKF，结果谁都没起：
    #   /odometry/filtered 发布者 0 个 → odom→base_footprint TF 缺失
    #   → AMCL 生命周期 active 却完全不工作（不发 /amcl_pose、不发
    #   map→odom），问题极难定位，因为它不报任何错。
    #   已在 bringup.launch.py 里补齐 IMU 偏置 + EKF + imu_link TF，
    #   「bringup 已提供」这句话现在才真正成立。
    # ==========================================================
    start_odom_helpers = launch.substitutions.LaunchConfiguration(
        'start_odom_helpers', default='false')
    odom_helpers_condition = launch.conditions.IfCondition(start_odom_helpers)
    
    # URDF → TF（发布 base_footprint→base_link→laser_link 静态变换）
    # 必须最先启动，否则 costmap/AMCL 找不到 robot_base_frame
    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [fishbot_bringup_dir, '/launch', '/urdf2tf.launch.py']),
        condition=odom_helpers_condition,
    )
    
    # odom→base_footprint TF广播
    odom2tf = Node(
        package='fishbot_bringup',
        executable='odom2tf',
        output='screen',
        condition=odom_helpers_condition,
    )
    
    # EKF节点（用于融合里程计和IMU）
    # 注意：不要添加 remapping ('odometry/filtered' -> 'odom')，
    # 因为 ekf.yaml 中 odom0: /odom_ekf，EKF 会同时读写 /odom 导致反馈环路！
    # 正确做法：odom2tf 节点直接订阅 EKF 的输出话题 /odometry/filtered
    # ----------------------------------------------------------
    # 必须跟随 odom_helpers_condition：bringup 已经起了一份 EKF，
    # 这里再起一份会出现两个独立滤波器同时往 /odometry/filtered
    # 写数据。odom2tf 收到的是两个滤波器交错的结果 → odom→base_link
    # 在两者估计之间反复跳变 → AMCL 观测模型被污染，收敛极慢甚至发散。
    # 实测：重复启动时 /odometry/filtered 有 2 个 Publisher。
    #
    # 注意 IMU 静态 TF：bringup 里有独立的 publish_imu_tf 开关，
    # 这里不重复发布。若用 start_odom_helpers:=true 单独调试导航，
    # 需要自己确保 base_link→imu_link 存在，否则 EKF 会静默丢弃 IMU。
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[{'use_sim_time': False}, ekf_config_path],
        condition=odom_helpers_condition,
    )
    
    # 协方差注入节点 —— 给 ESP32 /odom 补上合理协方差再转发给 EKF
    odom_cov_injector = Node(
        package='fishbot_bringup',
        executable='odom_covariance_injector.py',
        output='screen',
        condition=odom_helpers_condition,
    )
    
    # IMU陀螺偏置校准节点（修正MPU6050的wz漂移）
    # 同样必须跟随 odom_helpers_condition：两个校准节点各自估一套偏置，
    # 会把两份不同的 /imu_corrected 交替塞给 EKF，融合结果来回跳。
    imu_bias_remover = Node(
        package='fishbot_bringup',
        executable='imu_bias_remover',
        output='screen',
        condition=odom_helpers_condition,
    )

    # ==========================================================
    # AMCL 自动全局定位
    # ----------------------------------------------------------
    # 不再写死「机器人就在地图原点」这个先验：nav2_params.yaml 里
    # set_initial_pose=False，由本节点在 AMCL 就绪后调用
    # /reinitialize_global_localization，让粒子云撒满整张地图，
    # 随小车移动自动收敛到真实位姿。
    #
    # 额外周期性调用 /request_nomotion_update：小车静止时 AMCL 默认
    # 不更新粒子，不动就永远不收敛（参考 wheeltec 的
    # nomotion_update_period_sec: 0.5）。
    #
    # 若期间外部（RViz / 巡逻模块 / 网页）发了 /initialpose，
    # 本节点自动停止干预。
    # ==========================================================
    auto_localization = launch.substitutions.LaunchConfiguration(
        'auto_localization', default='true')
    auto_localization_prior = launch.substitutions.LaunchConfiguration(
        'auto_localization_use_prior', default='false')
    auto_localization_delay = launch.substitutions.LaunchConfiguration(
        'auto_localization_delay_sec', default='3.0')
    auto_localization_timeout = launch.substitutions.LaunchConfiguration(
        'auto_localization_timeout_sec', default='120.0')

    auto_localizer = Node(
        package='fishbot_navigation2',
        executable='auto_localizer.py',
        name='auto_localizer',
        output='screen',
        parameters=[{
            # 必须转成 bool。LaunchConfiguration 求值出来是字符串，
            # 直接当参数会写成 use_sim_time: 'False'（字符串），
            # 而节点把该参数声明为 bool —— rclpy 会拒绕这个类型。
            'use_sim_time': launch_ros.parameter_descriptions
            .ParameterValue(use_sim_time, value_type=bool),
            'auto_global_localization': launch_ros.parameter_descriptions
            .ParameterValue(auto_localization, value_type=bool),
            'use_initial_pose_prior': launch_ros.parameter_descriptions
            .ParameterValue(auto_localization_prior, value_type=bool),
            'startup_delay_sec': launch_ros.parameter_descriptions
            .ParameterValue(auto_localization_delay, value_type=float),
            'timeout_sec': launch_ros.parameter_descriptions
            .ParameterValue(auto_localization_timeout, value_type=float),
        }],
    )

    return launch.LaunchDescription([
        # 启动横幅：让日志里一眼能确认「这是带自动全局定位的版本」，
        # 而不是靠猜。本项目曾因看不出导航栈版本而白查很久。
        launch.actions.LogInfo(msg=(
            '\n'
            '================================================================\n'
            ' fishbot 导航栈（含 AMCL 自动全局定位）\n'
            '   ├─ set_initial_pose=False，不假设车在地图原点\n'
            '   ├─ auto_localizer 调用 /reinitialize_global_localization\n'
            '   │    让粒子云撒满全图，随移动自动收敛\n'
            '   └─ 静止时周期调 /request_nomotion_update 帮助收敛\n'
            ' 收敛判据：σ_xy ≤ 0.08m 且 σ_yaw ≤ 0.05rad 持续 3s\n'
            ' 未收敛时：缓慢原地转动小车（30~60°/秒）效果最好\n'
            '================================================================')),
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),
        launch.actions.DeclareLaunchArgument(
            'auto_localization', default_value='true',
            description='导航启动后自动做 AMCL 全局定位（不需要人工给初始位姿）'),
        launch.actions.DeclareLaunchArgument(
            'auto_localization_use_prior', default_value='false',
            description='不发全局散布，改用固定粗先验（仅当确定车总在同一点启动时用）'),
        launch.actions.DeclareLaunchArgument(
            'auto_localization_delay_sec', default_value='3.0',
            description='启动后多久开始尝试全局定位'),
        launch.actions.DeclareLaunchArgument(
            'auto_localization_timeout_sec', default_value='120.0',
            description='全局定位超时时间，超时则告警并停止自动干预'),
        launch.actions.DeclareLaunchArgument(
            'start_odom_helpers', default_value='false',
            description='是否由本 launch 启动 urdf2tf/odom2tf/协方差注入。'
                        '默认 false（认为 bringup 已启动），避免重复发布'),

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

        # AMCL 自动全局定位（全局散布 → 静止周期更新 → 收敛告警）
        auto_localizer,

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
