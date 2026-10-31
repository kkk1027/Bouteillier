import os
import shutil
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import conditions as launch_conditions
from launch import substitutions as launch_substitutions
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _make_microros_agent():
    """构造 micro_ros_agent 的启动动作。

    优先使用工作空间编译出的 ROS 包；若没有（例如本机跳过了该包编译），
    回退到系统里的 snap 版可执行文件 micro-ros-agent。
    否则 ros2 launch 会直接报 'package micro_ros_agent not found' 并退出，
    网页端表现为「连接失败 (退出码: 1)」。
    """
    try:
        get_package_share_directory('micro_ros_agent')
        return launch_ros.actions.Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['udp4', '--port', '8888'],
            output='screen'
        )
    except Exception:
        pass

    snap_agent = shutil.which('micro-ros-agent')
    if snap_agent:
        return launch.actions.ExecuteProcess(
            cmd=[snap_agent, 'udp4', '--port', '8888'],
            output='screen'
        )

    return launch.actions.LogInfo(
        msg='⚠️ 未找到 micro_ros_agent（ROS 包或 snap 版），ESP32 将无法接入'
    )


def _make_voice_control(enable_motion: str, condition=None):
    """构造语音控制节点的启动动作。

    依赖 fishbot_voice 包。若该包未编译（例如只选了部分包编译），
    仅打印警告，不阻断 bringup 其余部分启动。

    enable_motion 为字符串 'true'/'false'，由 voice_control.launch.py
    里的 ParameterValue(value_type=bool) 转成真正的 bool。
    condition 用于 enable_voice:=false 时跳过本项。
    """
    try:
        voice_dir = get_package_share_directory('fishbot_voice')
    except Exception:
        return launch.actions.LogInfo(
            msg='⚠️ 未找到 fishbot_voice 包（可能未编译），语音控制不可用。'
                '执行 colcon build --packages-select fishbot_voice 后可启用',
            condition=condition,
        )

    return launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [voice_dir, '/launch', '/voice_control.launch.py']),
        launch_arguments={
            'enable_motion': enable_motion,
        }.items(),
        condition=condition,
    )


def _make_odom_ekf_chain(enable_ekf, publish_imu_tf, use_sim_time,
                         imu_x, imu_y, imu_z, imu_roll, imu_pitch, imu_yaw):
    """构造完整的里程计融合链：imu_link TF + imu_bias_remover + EKF。

    ============================================================
    为什么这三样必须一起出现在 bringup 里
    ============================================================
    odom2tf 订阅的是 /odometry/filtered，而这个话题只有 EKF 会发。
    EKF 不起来 → /odometry/filtered 没有任何发布者 → odom→base_footprint
    这条 TF 永远不存在 → AMCL 拿不到里程计先验，转不了激光，
    既不发 /amcl_pose 也不发 map→odom。

    表现就是：AMCL 生命周期明明是 active，却完全「不工作」。

    历史坑（2026-10-31 实机排查）：
    navigation2.launch.py 把 EKF/odom2tf/协方差注入打包在
    start_odom_helpers 条件里，默认 false，注释理由是
    「bringup 已经起了一份 EKF，重复起会有两个滤波器」。
    但当时的 bringup 里根本没有 ekf_node —— 两边都以为对方起了，
    结果谁都没起。现在 bringup 真正补齐了这一环，那条注释才成立。

    ============================================================
    为什么必须补 base_link → imu_link 静态 TF
    ============================================================
    ESP32 固件（fishbot_motion_control/src/main.cpp:268）发的
    /imu_raw 的 frame_id 是 "imu_link"，但 URDF 里只有
    base_footprint / base_link / laser_link，没有 imu_link。

    robot_localization 在把 IMU 数据塞进滤波器之前，会先查
    imu_link → base_footprint 的 TF；查不到就丢弃这条测量，
    并刷 "Could not obtain transform from imu_link to base_footprint"。

    后果很隐蔽：EKF 正常输出 /odometry/filtered，只是完全没用上陀螺仪
    —— 朝向只能靠编码器积分，打滑时就会偏，而且不会有任何报错中断。

    安装偏移默认全 0（MPU6050 在控制板中心，与 base_link 视为重合），
    需要时可以按实际位置调。

    ============================================================
    参数
    ============================================================
    enable_ekf      总开关。设 false 时本函数不产出任何动作
    publish_imu_tf  是否补 imu_link 静态 TF（IMU 融合必需）
    use_sim_time    透传给 EKF
    """
    condition = launch_conditions.IfCondition(enable_ekf)
    actions = []

    # ---------------- 1. base_link → imu_link 静态 TF ----------------
    # 必须最先起：EKF 第一帧 IMU 就要查这个变换。
    actions.append(launch_ros.actions.Node(
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
        condition=launch_conditions.IfCondition(publish_imu_tf),
    ))

    # ---------------- 2. IMU 偏置校准 ----------------
    # /imu_raw → /imu_corrected。ekf.yaml 里 imu0 订阅的是后者，
    # 没有本节点 EKF 就完全收不到陀螺数据。
    # 本节点保持原始时间戳和 frame_id（见 imu_bias_remover.cpp:152）。
    try:
        get_package_share_directory('fishbot_bringup')
        actions.append(launch_ros.actions.Node(
            package='fishbot_bringup',
            executable='imu_bias_remover',
            name='imu_bias_remover',
            output='screen',
            condition=condition,
        ))
    except Exception:
        actions.append(launch.actions.LogInfo(
            msg='⚠️ 未找到 imu_bias_remover，/imu_corrected 将不存在，'
                'EKF 只能靠编码器积分估计朝向（打滑会偏）',
            condition=condition,
        ))

    # ---------------- 3. EKF 融合 ----------------
    # 输入：odom0=/odom_ekf（协方差注入后的编码器里程计）
    #       imu0=/imu_corrected（陀螺 vyaw）
    # 输出：/odometry/filtered → odom2tf 转成 odom→base_footprint TF
    #
    # 注意 ekf.yaml 里 publish_tf: false —— TF 由 odom2tf 独占发布，
    # 否则两者重复广播同一条 TF，位姿会跳变甚至镜像翻转。
    try:
        nav_dir = get_package_share_directory('fishbot_navigation2')
        ekf_config = os.path.join(nav_dir, 'config', 'ekf.yaml')
        actions.append(launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            condition=condition,
        ))
    except Exception as exc:                       # noqa: BLE001
        actions.append(launch.actions.LogInfo(
            msg='⚠️ 无法启动 EKF（%s）。/odometry/filtered 不会有任何'
                '发布者，odom→base_footprint TF 缺失，AMCL 将无法工作'
                % exc,
            condition=condition,
        ))

    return actions


def generate_launch_description():
    fishbot_bringup_dir = get_package_share_directory(
        'fishbot_bringup')
    ydlidar_ros2_dir = get_package_share_directory(
        'ydlidar')

    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [fishbot_bringup_dir, '/launch', '/urdf2tf.launch.py']),
    )

    # 协方差注入节点 —— 给 ESP32 /odom 补上合理协方差再转发给 EKF
    odom_cov_injector = launch_ros.actions.Node(
        package='fishbot_bringup',
        executable='odom_covariance_injector.py',
        output='screen'
    )

    odom2tf = launch_ros.actions.Node(
        package='fishbot_bringup',
        executable='odom2tf',
        output='screen'
    )

    microros_agent = _make_microros_agent()

    ros_serial2wifi =  launch_ros.actions.Node(
        package='ros_serial2wifi',
        executable='tcp_server',
        parameters=[{'serial_port': '/tmp/tty_laser'}],
        output='screen'
    )

    # ★ 必须显式传 ydlidar_params_file。
    #
    # 千万不要写成默认、也不要改回叫 params_file：bringup 里同时 include
    # 了 voice_control.launch.py，它声明了同名的 params_file（默认
    # voice_control.yaml）。launch 配置是全局的，语音先生效，ydlidar 的
    # 默认值会被静默忽略 —— 于是 ydlidar_node 拿着语音的 yaml 启动，
    # 参数全部回落默认（port=/dev/ydlidar），立即失败、respawn 无限重试，
    # /scan 永久消失。详见 ydlidar_launch.py 里的注释。
    ydlidar_params = os.path.join(ydlidar_ros2_dir, 'params', 'ydlidar.yaml')
    ydlidar = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ydlidar_ros2_dir, '/launch', '/ydlidar_launch.py']),
        launch_arguments={
            'ydlidar_params_file': ydlidar_params,
        }.items(),
    )

    # 使用 TimerAction 启动后 5 秒执行 ydlidar 节点
    ydlidar_delay = launch.actions.TimerAction(period=5.0, actions=[ydlidar])

    # ---------------- 语音控制 ----------------
    # enable_voice        : 是否启动语音控制节点（默认启动）
    # enable_voice_motion : 语音是否真的驱动底盘
    #   true  = 实车模式，喊一句车就动（默认，满足「接上就能用」）
    #   false = 干跑模式，只解析打日志，不动车
    # 语音节点本身不依赖其他节点，且未收到语音事件前不会发任何指令，
    # 所以直接随 bringup 启动，不做延迟。
    enable_voice_arg = launch.actions.DeclareLaunchArgument(
        'enable_voice', default_value='true',
        description='是否启动语音控制节点')
    enable_voice_motion_arg = launch.actions.DeclareLaunchArgument(
        'enable_voice_motion', default_value='true',
        description='语音是否驱动底盘：true=实车，false=干跑只打日志')

    voice_control = _make_voice_control(
        launch_substitutions.LaunchConfiguration('enable_voice_motion'),
        condition=launch_conditions.IfCondition(
            launch_substitutions.LaunchConfiguration('enable_voice')))

    # ---------------- 里程计融合链参数 ----------------
    # enable_ekf      : 是否启动 EKF 融合。默认 true ——
    #                   导航（AMCL/costmap）离不开 odom→base_footprint TF，
    #                   而这条 TF 只有 EKF 起来才会有。
    # publish_imu_tf  : 是否补 base_link→imu_link 静态 TF。
    #                   URDF 里没有 imu_link，而 ESP32 的 IMU frame_id
    #                   就是 imu_link，缺了它 EKF 会静默丢弃全部 IMU。
    # imu_*           : IMU 相对 base_link 的安装偏移，默认重合。
    # use_sim_time    : 透传给 EKF。
    enable_ekf_arg = launch.actions.DeclareLaunchArgument(
        'enable_ekf', default_value='true',
        description='是否启动 imu_bias_remover + EKF（导航必需）')
    publish_imu_tf_arg = launch.actions.DeclareLaunchArgument(
        'publish_imu_tf', default_value='true',
        description='是否发布 base_link→imu_link 静态 TF（IMU 融合必需）')
    use_sim_time_arg = launch.actions.DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='是否使用仿真时间')

    imu_offset_args = [
        launch.actions.DeclareLaunchArgument(
            'imu_' + name, default_value='0.0',
            description='IMU 相对 base_link 的 %s' % name)
        for name in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
    ]

    odom_ekf_chain = _make_odom_ekf_chain(
        launch_substitutions.LaunchConfiguration('enable_ekf'),
        launch_substitutions.LaunchConfiguration('publish_imu_tf'),
        launch_substitutions.LaunchConfiguration('use_sim_time'),
        *(launch_substitutions.LaunchConfiguration('imu_' + name)
          for name in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')))

    return launch.LaunchDescription([
        enable_voice_arg,
        enable_voice_motion_arg,
        enable_ekf_arg,
        publish_imu_tf_arg,
        use_sim_time_arg,
        *imu_offset_args,
        urdf2tf,

        # ★ 顺序说明
        #   1) URDF TF 必须先有，否则 imu_link 挂不到 base_link 上。
        #   2) 然后是 imu_link 静态 TF（latched，越早越好）。
        #   3) imu_bias_remover 产出 /imu_corrected。
        #   4) odom_cov_injector 产出 /odom_ekf。
        #   5) odom2tf 订阅 /odometry/filtered。
        #   6) EKF 吃 /odom_ekf + /imu_corrected，吐 /odometry/filtered。
        # 4~6 都是 subscriber，谁先起来都只是等数据，不会出错；
        # 真正有先后要求的是 1→2，所以把 urdf2tf 放在最前面。
        *odom_ekf_chain,

        odom_cov_injector,
        odom2tf,
        microros_agent,
        ros_serial2wifi,
        ydlidar_delay,
        voice_control,
    ])
