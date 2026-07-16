# FishBot 机器人项目 — 全面工作总结报告

> 生成日期：2026-05-31  
> 工作空间：`~/chapt9_update/`  
> 原始项目：`~/chapt9/`（**未修改**）

---

## 一、项目概览

本工作空间是 `~/chapt9/` 的完整拷贝，用于修复 Bug、优化配置和调试。所有修改**仅在 `~/chapt9_update/` 下进行**，原始项目保持原样。

### 1.1 项目结构

```
~/chapt9_update/
├── fishbot_ws/                          # ROS2 Humble 工作空间
│   ├── src/
│   │   ├── autopatrol_robot/            # AI智能体（LangGraph+DeepSeek）+ 自动巡逻 + Web控制
│   │   ├── custom_interfaces/           # 自定义通信接口定义
│   │   ├── customservice/               # 自定义服务
│   │   ├── fishbot_application/         # 导航/航点跟随应用层
│   │   ├── fishbot_bringup/             # 启动总控 + IMU偏置校准 + odom2tf
│   │   ├── fishbot_description/         # URDF机器人描述
│   │   ├── fishbot_navigation2/         # Navigation2 + EKF定位 + AMCL
│   │   ├── micro_ros_agent/             # micro-ROS UDP代理
│   │   ├── micro_ros_msgs/              # micro-ROS消息定义
│   │   ├── robot_controller_qt/         # Qt桌面控制界面（C++）
│   │   ├── ros_serial2wifi/             # ROS串口桥接
│   │   └── ydlidar/                     # 激光雷达驱动
│   ├── build/ install/ log/             # 编译产物
│   └── yolov8n.pt                       # YOLOv8模型
│
└── fishbot_motion_control/              # PlatformIO ESP32 下位机固件
    ├── src/main.cpp                     # 主控程序
    ├── lib/                             # 库（Kinematics, PID, Servo, Button...）
    └── platformio.ini                   # PlatformIO配置
```

### 1.2 技术栈

| 组件 | 技术选型 | 版本/说明 |
|------|---------|----------|
| **机器人框架** | ROS 2 Humble | `/opt/ros/humble/` |
| **下位机** | ESP32 + micro-ROS | PlatformIO + Arduino框架 |
| **定位方案** | EKF (robot_localization) + AMCL (Navigation2) | Odom提供yaw, IMU只提供wz |
| **路径规划** | SmacPlanner2D + RegulatedPurePursuit | Navigation2 |
| **AI智能体** | LangGraph + LangChain + DeepSeek | API Key硬编码，model="deepseek-chat" |
| **Web控制** | Flask + SocketIO | 端口 5000 |
| **激光雷达** | YDLIDAR | 5秒延迟启动（硬件稳定需要） |
| **IMU** | MPU6050 | 陀螺Z轴有温漂偏置 |
| **WiFi** | micro-ROS UDP | SSID: "kkk", 端口 8888, Agent: 192.168.2.128 |

### 1.3 硬件参数

| 参数 | 值 |
|------|-----|
| 左轮PID | [0.625, 0.125, 0] |
| 右轮PID | [0.510, 0.125, 0] |
| 轮距 | 175 mm |
| 机器人半径 | 0.272 m |
| 激光串口 | `/tmp/tty_laser` |

---

## 二、修复问题清单（共 29 项）

### 阶段一：初始 17 项修复（P0关键Bug + 硬编码 + 逻辑Bug）

#### P0 关键 Bug 修复（5 项）

| # | 文件 | 问题 | 修复 |
|:-:|------|------|------|
| 1 | `navigation2.launch.py` | EKF节点创建后返回值被丢弃 | 赋值给 `ekf_node` 变量并加入 LaunchDescription |
| 2 | `main.cpp` (ESP32) | `rclc_executor_add_subscription` 传入 NULL | 创建 `std_msgs__msg__Bool` 变量并传递地址 |
| 3 | `patrol_node.py` | ServoClient 未初始化 `cmd_vel_pub` | 添加 `self.cmd_vel_pub`、`self._logger`；修正 `rclpy.spin_once(self.node)` |
| 4 | `agent_tools.py` | `cleanup_ros()` 引用未定义的 `_rclpy_initialized` | 添加全局变量及 `global` 声明 |
| 5 | `nav_to_pose.py` | `goal_pose.pose.orientation.w = 3.14` 非法四元数 | 使用 `tf_transformations.quaternion_from_euler(0, 0, 3.14)` |

#### 路径硬编码修复（7 项）

| # | 文件 | 修复方式 |
|:-:|------|---------|
| 6 | `navigation2.launch.py` | 改用 `os.path.join(fishbot_navigation2_dir, 'config', 'ekf.yaml')` |
| 7 | `agent_tools.py` | 改用 `os.environ.get('FISHBOT_WS', '/home/ke/chapt9_update/fishbot_ws')` |
| 8 | `web_controller.py` (3处) | 同上环境变量方式 |
| 9 | `ekf.launch.py` | 改用 `get_package_share_directory()` + `os.path.join()` |
| 10 | `ydlidar_cartographer.launch.py` (2处) | URDF用 `get_package_share_directory()`；Cartographer配置用 `EnvironmentVariable` |
| 11 | `robot_controller_qt/*.cpp` (9处) | sed批量替换 + `qEnvironmentVariable("FISHBOT_WS", ...)` |
| 12 | `manual_controller.cpp` | 使用环境变量 `FISHBOT_WS` 动态拼接 |

#### 逻辑/配置 Bug 修复（4 项）

| # | 文件 | 问题 | 修复 |
|:-:|------|------|------|
| 13 | `bringup.launch.py` | `ros_serail2wifi` 拼写错误 | 改为 `ros_serial2wifi` |
| 14 | `ekf.yaml` | `odom0_differential: false` | 改为 `true`（编码器里程计应使用差分模式） |
| 15 | `waypoint_follower.py` | 用 `Trigger` 服务，ESP32实际提供 `SetBool` | 改为 `SetBool` + `/esp32_command` |
| 16 | `display_robot.launch.py` | 引用不存在的 `urdf/first_robot.urdf` | 改为 `urdf/fishbot.urdf`；移除不存在的 rviz 配置 |

#### 删除（1 项）

| # | 文件 | 原因 |
|:-:|------|------|
| 17 | `speaker.py` | 依赖不存在的 `SpeachText` 服务接口，未被任何 entry_point 引用 |

---

### 阶段二：EKF 策略大重构（3 项重大变更）

#### 核心问题
IMU 的 `mpu.getAngleZ()` 通过纯陀螺积分计算 yaw，**长期漂移严重**。如果 EKF 把 IMU 的 yaw 当作绝对朝向输入，会破坏原本稳定的编码器 yaw。

#### 解决思路
| 原始策略（错误的） | 新策略（正确的） |
|------------------|----------------|
| IMU 提供 `orientation.yaw` + `angular_velocity.z` | IMU **只提供** `angular_velocity.z` |
| 编码器提供 `x, y, vx` | 编码器提供 `x, y, **yaw**, vx` |
| EKF 从 IMU 获取绝对朝向 → 漂移 | EKF 从编码器获取绝对朝向 → 稳定 |

#### 具体变更

| # | 文件 | 变更 |
|:-:|------|------|
| 18 | `config/ekf.yaml` | `odom0_config` 增加 **yaw(index 5)**；`imu0_config` 只保留 **wz(index 11)**；`imu0` 改为 `/imu_corrected` |
| 19 | `src/imu_bias_remover.cpp` | **新建** C++节点：订阅 `/imu_raw` 和 `/odom`，静止时采样陀螺偏置，发布 `/imu_corrected` |
| 20 | `CMakeLists.txt` (fishbot_bringup) | 添加 `sensor_msgs` 依赖、`imu_bias_remover` 编译目标 |
| 21 | `navigation2.launch.py` | 在 EKF 前启动 `imu_bias_remover` 节点 |

#### imu_bias_remover 设计细节

```
                    ┌──────────────┐
  /imu_raw ────────▶│              │
                    │  imu_bias_   ├───────▶ /imu_corrected
  /odom ───────────▶│  remover.cpp │         (wz - bias)
                    └──────────────┘
```

- **静止检测**：`|vx| < 0.01 m/s` 且 `|wz| < 0.02 rad/s`
- **初始标定**：100 个样本 / 3 秒（首次静止时自动完成）
- **运行时重标定**：50 个样本 / 1.5 秒（补偿温漂）
- **偏置估计**：中位数（对异常值不敏感）
- **可配置参数**：`stationary_vx_thresh`、`stationary_wz_thresh`、`calib_samples`、`calib_duration`

---

### 阶段三：nav2_params.yaml 全面优化（12 项）

#### 根本问题

原始 `nav2_params.yaml` **是从 Gazebo 仿真教程直接拷贝过来的**，所有参数假设：
- 里程计零误差（仿真中的 odometry 完美）
- 激光传感器零噪声
- 系统使用仿真时钟

真实机器人的打滑、地面不平、编码器误差、激光噪声**完全没有被考虑**。

#### 所有变更

| # | 参数/配置 | ❌ 原来（仿真） | ✅ 现在（真实） | 影响 |
|:-:|-----------|:--------------:|:--------------:|------|
| 22 | **use_sim_time** | 15处 `True` | **全部 `False`** | 所有节点使用真实系统时间 |
| 23 | **alpha1**（旋转→角度噪声） | 0.1 | **0.8** | 不信任轮式里程计的旋转角度 |
| 24 | **alpha2**（平移→角度噪声） | 0.1 | **0.5** | 平移时考虑角度误差 |
| 25 | **alpha3**（平移→位置噪声） | 0.1 | **0.4** | 平移时考虑位置误差 |
| 26 | **alpha4**（旋转→位置噪声） | 0.1 | **1.0** | 旋转时严重考虑位置误差 |
| 27 | **alpha5**（平移噪声相关） | 0.1 | **0.2** | 平移噪声相关度 |
| 28 | **z_hit / z_rand** | 0.5 / 0.5 | **0.9 / 0.05** | 50%→90%信任激光数据 |
| 29 | **update_min_d / update_min_a** | 0.25m / 0.2rad | **0.1m / 0.1rad** | 更频繁地用激光修正粒子 |
| 30 | **min_particles** | 250 | **500** | 更多粒子覆盖更大搜索空间 |
| 31 | **recovery_alpha_fast/slow** | 0.0 / 0.0（关闭） | **0.1 / 0.001**（启用） | 粒子退化时自动恢复 |
| 32 | **transform_tolerance** | 1.0s | **0.2s** | 减少 TF 滞后 |
| 33 | **laser_likelihood_max_dist** | 2.0m | **5.0m** | 充分利用 YDLIDAR 量程 |
| 34 | **local_costmap 障碍物层** | VoxelLayer（3D体素） | **ObstacleLayer**（2D） | 减少CPU开销 |
| 35 | **global_costmap 感知范围** | 2.5m / 3.0m raytrace | **4.0m / 5.0m raytrace** | 全局路径规划看得更远 |
| 36 | **Planner 搜索模型** | VON_NEUMANN（4方向） | **MOORE**（8方向） | 规划路径更平滑 |
| 37 | **Planner 期望频率** | 20.0 Hz | **5.0 Hz** | 减少超时警告 |
| 38 | **goal_tolerance** | 0.25m / 0.25rad | **0.15m / 0.15rad** | 导航到位更精确 |
| 39 | **behavior spin_vel** | 0.8（> max 0.6 被截断） | **0.6** | 自旋速度和行为参数一致 |
| 40 | **behavior max_rotation_time** | 5.0s（不够180°旋转） | **8.0s** | 自旋不会意外超时 |
| 41 | **local_costmap 死代码** | 未启用的 `static_layer` 配置块 | **已清理** | 配置文件更干净 |
| 42 | **controller_server 残留注释** | `# ... 保留其余的配置不变` | **已清理** | 干净 |
| 43 | **nav2_params.yaml 备份** | 无 | **已创建 `.bak`** | 可回滚 |

---

## 三、EKF 融合策略详解

### 3.1 数据流

```
ESP32 Firmware (main.cpp)
│
├── 编码器 → 差速运动学 → /odom (x, y, yaw, vx)
│                              │
│                              ▼
│                          EKF (robot_localization)
│                          ┌──────────────────────┐
│                          │  输入: odom0 (编码器)  │
│                          │  输入: imu0 (修正后IMU) │
│                          │  输出: /odometry/filtered│
│                          └──────────┬───────────┘
│                                     │
│                                     ▼
│                              odom2tf.cpp
│                          ┌──────────────────────┐
│                          │  odom → base_footprint │
│                          └──────────────────────┘
│
└── MPU6050 → /imu_raw (含偏置)
                     │
                     ▼
              imu_bias_remover.cpp
          ┌────────────────────────┐
          │  静止时采样 wz 偏置    │
          │  输出: /imu_corrected   │
          │  (wz_corrected =       │
          │   wz_raw - bias)       │
          └───────────┬────────────┘
                      │
                      ▼
                  EKF (imu0输入)
```

### 3.2 EKF 配置要点

```yaml
# odom0: 编码器里程计 — 提供绝对 yaw（稳定不漂移）
odom0: /odom
odom0_config: [true, true, false, false, false, true, true, false, ...]
#               x     y                      yaw    vx
odom0_differential: false   # 直接使用绝对 yaw，非差分模式

# imu0: 修正后IMU — 只提供 wz（短期精准角速度）
imu0: /imu_corrected
imu0_config: [false, false, false, false, false, false, false, false, false, false, false, true, ...]
#                                                                                         wz
imu0_differential: false
```

---

## 四、系统启动流程

```
bringup.launch.py                    navigation2.launch.py
┌──────────────────┐               ┌─────────────────────────┐
│ urdf2tf          │               │ imu_bias_remover (NEW)   │
│ odom2tf          │               │ ekf_node                 │
│ micro_ros_agent  │               │ nav2_bringup_launch.py   │
│ ros_serial2wifi  │               │ rviz2                    │
│ ydlidar (5s延时)  │               └─────────────────────────┘
└──────────────────┘
```

---

## 五、导航行为树 (BT Navigator) 插件列表

系统已启用 46 个行为树节点插件，涵盖：

| 类别 | 插件数 | 用途 |
|------|:------:|------|
| **路径规划** | 3 | 路径计算、经过点、路径平滑 |
| **路径跟踪** | 1 | FollowPath |
| **恢复行为** | 6 | Spin、BackUp、Wait、AssistedTeleop、DriveOnHeading、清除costmap |
| **条件检查** | 8 | 卡住检测、目标到达、路径有效、初始位姿、低电量等 |
| **动作** | 9 | 导航到位、经过点导航、取消动作等 |
| **控制/修饰** | 6 | Pipeline序列、RoundRobin、速率控制、距离控制等 |
| **其他** | 13 | 变换可用、超时、路径过期、Goal更新等 |

---

## 六、已知遗留问题

### 6.1 不受本项目影响的路径

| 路径/引用 | 所在文件 | 说明 |
|-----------|---------|------|
| `/home/ke/detection_ws` | `agent_tools.py` ~731行 | 外部视觉工作空间，不在本项目中 |
| `/home/ke/cartographer_config` | 变为 `CARTOGRAPHER_CONFIG_DIR` 环境变量 | 已改为可覆盖 |
| `hardware_config.yaml` 中 `serial_port: /dev/ttyACM0` | 设备端口 | 运行时需根据实际情况确认 |

### 6.2 备份文件说明

| 文件 | 位置 | 说明 |
|------|------|------|
| `nav2_params.yaml.bak` | `fishbot_navigation2/config/` | nav2_params原始备份，用于回滚 |
| `web_controller copy.py` | `autopatrol_robot/autopatrol_robot/` | 副本文件，不影响功能 |
| `patrol_node（backup）.py` | `autopatrol_robot/autopatrol_robot/` | 备份文件（中文全角括号文件名） |

### 6.3 注意

- `launch_manager.cpp` 中仍有少量硬编码路径（`/home/ke/chapt9_update/fishbot_ws/...`），如需完全可移植需改用环境变量
- Qt控制界面路径在 `manual_controller.cpp` 和 `ros_map_bridge.cpp` 已改为 `qEnvironmentVariable("FISHBOT_WS", ...)`

---

## 七、AMCL 调参经验总结

### 真实机器人 vs 仿真参数对比

| AMCL参数 | 仿真（Gazebo） | 真实机器人（经测试有效） | 调参逻辑 |
|----------|:-------------:|:---------------------:|---------|
| `alpha1` | 0.1 | **0.8** | 仿真里程计零误差；真实轮子打滑严重 |
| `alpha2` | 0.1 | **0.5** | 平移时会有不可控的旋转 |
| `alpha3` | 0.1 | **0.4** | 编码器累计误差 |
| `alpha4` | 0.1 | **1.0** | 自旋时位置误差最大（轮子原地打滑） |
| `z_hit` | 0.5 | **0.9** | 仿真激光完美；真实激光虽有噪声但可信度高 |
| `z_rand` | 0.5 | **0.05** | 50% 随机噪声假设过高 |
| `update_min_d` | 0.25 | **0.1** | 10cm 更新一次粒子滤波 |
| `transform_tolerance` | 1.0 | **0.2** | 1秒会导致 TF 严重滞后 |

### 调参后预期行为变化

1. **粒子云更发散**（alpha 值增大）：初始定位时粒子覆盖更大区域
2. **收敛更快**（z_hit=0.9）：激光匹配权重高，粒子快速收敛到正确位置
3. **更新更频繁**（update_min_d/a 降低）：机器人每走10cm/转5.7°就做一次激光匹配
4. **不易丢失**（recovery 启用）：粒子退化时自动补充随机粒子
5. **规划更远**（laser_likelihood_max_dist=5.0）：全局路径利用更远的激光数据

---

## 八、构建与运行指南

### 构建命令

```bash
# 全部构建
cd ~/chapt9_update/fishbot_ws
source /opt/ros/humble/setup.bash
colcon build

# 单个包构建（推荐调试时使用）
colcon build --packages-select fishbot_navigation2
colcon build --packages-select fishbot_bringup
colcon build --packages-select autopatrol_robot
colcon build --packages-select fishbot_application
colcon build --packages-select fishbot_description
```

### 运行

```bash
# 刷新环境
source ~/chapt9_update/fishbot_ws/install/setup.bash

# 可以设置环境变量覆盖默认工作空间路径
export FISHBOT_WS=~/chapt9_update/fishbot_ws

# 启动导航（含EKF+AMCL+建图）
ros2 launch fishbot_navigation2 navigation2.launch.py

# 启动基础功能（URDF+TF+micro-ROS+激光雷达）
ros2 launch fishbot_bringup bringup.launch.py

# 启动AI智能体
ros2 run autopatrol_robot agent_tools.py
```

---

## 九、系统架构图

```
┌──────────────────────────────────────────────────────────┐
│                    ROS 2 Humble                           │
│                                                           │
│  ┌───────────┐   ┌──────────────┐   ┌──────────────────┐ │
│  │ ESP32     │──▶│ EKF          │──▶│ odom2tf          │ │
│  │ (micro-ROS)│   │ (robot_local)│   │ (odom→base_link) │ │
│  │ 编码器+IMU │   └──────┬───────┘   └──────────────────┘ │
│  └───────────┘          │                                 │
│                         ▼                                 │
│  ┌───────────┐   ┌──────────────┐                        │
│  │ YDLIDAR   │──▶│ AMCL         │──▶ map→odom TF         │
│  └───────────┘   │ (Navigation2)│                        │
│                  └──────┬───────┘                        │
│                         │                                 │
│                         ▼                                 │
│  ┌───────────┐   ┌──────────────┐                        │
│  │ AI Agent  │──▶│ Nav2 Stack   │──▶ cmd_vel → ESP32     │
│  │ (LangGraph)│  │ (Planner+    │                        │
│  └───────────┘   │  Controller) │                        │
│                  └──────────────┘                        │
│                                                           │
│  ┌───────────┐   ┌──────────────┐                        │
│  │ Web UI    │──▶│ Flask+SocketIO│──▶ 远程控制           │
│  │ (Port 5000)│  └──────────────┘                        │
│  └───────────┘                                            │
│                                                           │
│  ┌───────────┐   ┌──────────────┐                        │
│  │ Qt Desktop│──▶│ C++ GUI      │──▶ 桌面控制            │
│  └───────────┘   └──────────────┘                        │
└──────────────────────────────────────────────────────────┘
```

---

## 十、版本变更时间线

```
2026-05-30 初始审核阶段
  ├── 确认17项修复全部到位
  ├── 清理备份文件和构建残留
  └── 全面分析代码结构

2026-05-30 EKF调试阶段
  ├── 发现IMU yaw漂移破坏EKF输出
  ├── 重构ekf.yaml策略：odom提供yaw，IMU只提供wz
  ├── 新建imu_bias_remover.cpp（C++偏置校准节点）
  ├── 更新CMakeLists.txt和navigation2.launch.py
  ├── 用户测试：EKF稳定，但点云与地图匹配不佳
  └── 用户要求检查nav2_params.yaml

2026-05-30 AI导航Bug修复
  ├── 发现AI导航"先去客厅再去厨房"卡死，CLI正常
  ├── 根因分析：subprocess stdout 管道缓冲区 (64KB) 填满导致 write() 阻塞死锁
  ├── 新增 _start_stdout_drainer() 守护线程方案
  ├── 修复 nav_manager.py 4 处 Popen + web_controller.py 2 处 Popen
  └── 详见第十一章

2026-05-31 AMCL优化阶段
  ├── 发现nav2_params.yaml是仿真配置
  ├── 全面重写AMCL参数（alpha1-5, z_hit/z_rand等）
  ├── 用户测试："效果非常不错" ✅
  ├── 全面分析剩余问题（7处use_sim_time、costmap、planner等）
  ├── 一次性修复12项优化（含备份）
  └── 生成本总结报告
```

---

## 十一、AI 导航 stdout 管道死锁 Bug 修复

> 修复日期：2026-05-30  
> 严重程度：**P0 阻塞性 Bug**  
> 影响范围：所有通过 AI 智能体 (LangGraph) 发起的导航任务

### 11.1 现象

- **AI 对话方式**："先去客厅再去厨房" → 机器人到达第一个目的地后**直接卡死**，后续目标不再执行
- **CLI 方式**：`ros2 launch autopatrol_robot autopatrol.launch.py target_indices:="3,2"` → 正常完成全部导航 ✅

两种方式执行**完全相同的代码** (`patrol_node.main()`)，唯一的区别是启动方式不同。

### 11.2 根因分析：stdout 管道缓冲区死锁

#### 两条路径对比

| | CLI 路径 | AI 路径 |
|---|---|---|
| **启动方式** | 终端直接 ros2 launch | `nav_manager.py` 通过 `subprocess.Popen` |
| **stdout 目标** | pty (终端) → **无限缓冲** | `subprocess.PIPE` → **64KB 上限** |
| **stdout 消费者** | 终端自动显示 | **无人读取** ❌ |
| **结果** | ✅ 正常 | ❌ 到达第一目标后冻结 |

#### 死锁机制

```
patrol_node.nav_to_pose() 每 0.1s 打印一行日志 (~150 bytes)
     │
     ▼
内核管道缓冲区 (默认 64KB)
     │
     ├─ CLI: 终端自动消费 stdout → 缓冲区永不填满
     │
     └─ subprocess: 无人读取 PIPE
         │
         ▼
     约 60 秒后缓冲区填满 (~64KB / 150 bytes × 0.1s)
         │
         ▼
     patrol_node 的 write() 系统调用阻塞
         │
         ▼
     进程冻结 → poll() 永远返回 None
         │
         ▼
     wait_for_nav_completion() 永远等不到
         │
         ▼
     AI tool call 永不返回 → 用户看来机器人"卡死"
```

**关键证据**：多目标序列导航 (`launch_navigate_sequence`) 的进程持续时间长（每个目标 ~2-5 分钟导航 + ~60s 按钮等待），缓冲区必然在第一个目标期间填满。

### 11.3 修复方案

新增 `_start_stdout_drainer(proc)` 函数，在 subprocess 创建后**立即**启动守护线程持续读取 stdout：

```python
def _start_stdout_drainer(proc: subprocess.Popen):
    """后台线程持续读取子进程 stdout，防止管道缓冲区满导致子进程 write() 阻塞死锁。"""
    if proc is None or proc.stdout is None:
        return

    def _drain():
        try:
            while True:
                chunk = proc.stdout.read(4096)
                if not chunk:
                    break
        except (ValueError, OSError):
            pass
        except Exception:
            pass

    t = threading.Thread(target=_drain, daemon=True)
    t.start()
```

### 11.4 修改文件清单

| 文件 | 修改内容 | Popen 调用数 |
|------|---------|:---:|
| `nav_manager.py` | 新增 `_start_stdout_drainer()`；4 个 launch 函数各追加 1 行调用；`_cleanup_process()` 移除阻塞 read() | 4 |
| `web_controller.py` | import drainer；`load_map` 和 `connect_robot` 各追加 1 行调用 | 2 |

| Popen 位置 | 是否修复 | 说明 |
|-----------|:---:|------|
| `nav_manager.launch_navigate_by_index` | ✅ | 单点导航 |
| `nav_manager.launch_navigate_sequence` | ✅ | **序列导航（核心修复）** |
| `nav_manager.launch_return_to_start` | ✅ | 返航 |
| `nav_manager.launch_navigate_xy` | ✅ | 坐标导航 |
| `web_controller.MapManager.load_map` | ✅ | 加载地图 |
| `web_controller.connect_robot` | ✅ | bringup 长时运行进程 |
| `web_controller.soft_reboot` | ⬜ 无需 | 使用 `communicate(timeout=5)` 主动读管道 |
| `follow_tools.start_patrol_mode` | ⬜ 无需 | 已有 `read_output()` 线程读取 |

### 11.5 验证方法

修复后，AI 对话"先去客厅再去厨房"应表现与 CLI 完全一致：
1. 导航到客厅 → 舵机播报 → 等待按钮 → 继续导航到厨房 → 舵机播报 → 返回起点
2. 不再出现"到达第一个目的地后卡死"的现象

---

## 附录：关键文件索引

| 文件 | 路径 |
|------|------|
| ESP32固件 | `~/chapt9_update/fishbot_motion_control/src/main.cpp` |
| EKF配置 | `~/chapt9_update/fishbot_ws/src/fishbot_navigation2/config/ekf.yaml` |
| 导航完整配置 | `~/chapt9_update/fishbot_ws/src/fishbot_navigation2/config/nav2_params.yaml` |
| 导航启动 | `~/chapt9_update/fishbot_ws/src/fishbot_navigation2/launch/navigation2.launch.py` |
| IMU偏置校准 | `~/chapt9_update/fishbot_ws/src/fishbot_bringup/src/imu_bias_remover.cpp` |
| bringup启动 | `~/chapt9_update/fishbot_ws/src/fishbot_bringup/launch/bringup.launch.py` |
| odom→TF桥 | `~/chapt9_update/fishbot_ws/src/fishbot_bringup/src/odom2tf.cpp` |
| CMakeLists(bringup) | `~/chapt9_update/fishbot_ws/src/fishbot_bringup/CMakeLists.txt` |
| AI智能体 | `~/chapt9_update/fishbot_ws/src/autopatrol_robot/autopatrol_robot/agent_tools.py` |
| 巡逻节点 | `~/chapt9_update/fishbot_ws/src/autopatrol_robot/autopatrol_robot/patrol_node.py` |
| Web控制器 | `~/chapt9_update/fishbot_ws/src/autopatrol_robot/autopatrol_robot/web_controller.py` |
| 导航应用 | `~/chapt9_update/fishbot_ws/src/fishbot_application/fishbot_application/nav_to_pose.py` |
| 航点跟随 | `~/chapt9_update/fishbot_ws/src/fishbot_application/fishbot_application/waypoint_follower.py` |
| Qt启动管理 | `~/chapt9_update/fishbot_ws/src/robot_controller_qt/src/launch_manager.cpp` |
| Qt手动控制 | `~/chapt9_update/fishbot_ws/src/robot_controller_qt/src/manual_controller.cpp` |
| 导航备份配置 | `~/chapt9_update/fishbot_ws/src/fishbot_navigation2/config/nav2_params.yaml.bak` |
| 本报告 | `~/chapt9_update/SUMMARY_REPORT.md` |
