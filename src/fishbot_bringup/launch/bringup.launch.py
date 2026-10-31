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

    return launch.LaunchDescription([
        enable_voice_arg,
        enable_voice_motion_arg,
        urdf2tf,
        odom_cov_injector,
        odom2tf,
        microros_agent,
        ros_serial2wifi,
        ydlidar_delay,
        voice_control,
    ])
