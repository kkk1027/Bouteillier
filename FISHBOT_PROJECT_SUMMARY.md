# FishBot 机器人项目 — 完整项目总结

> **项目**: chapt9_update / fishbot_ws  
> **日期**: 2026-06-07  
> **原始项目**: ~/chapt9/（未修改，本 workspace 为完整拷贝+修复+优化）  
> **ROS 版本**: ROS 2 Humble  
> **下位机**: ESP32 (PlatformIO + Arduino + micro-ROS over WiFi UDP)  

---

## 目录

1. [项目总览](#1-项目总览)
2. [硬件架构](#2-硬件架构)
3. [软件包清单与功能](#3-软件包清单与功能)
4. [全系统数据流](#4-全系统数据流)
5. [核心链路深度解析](#5-核心链路深度解析)
6. [修复与优化清单（43 项）](#6-修复与优化清单43-项)
7. [启动流程](#7-启动流程)
8. [关键设计决策](#8-关键设计决策)
9. [已知遗留问题](#9-已知遗留问题)

---

## 1. 项目总览

FishBot 是一个基于 ROS 2 Humble 的**差速轮式移动机器人**，具备以下核心能力：

| 能力 | 实现方式 |
|------|---------|
| **SLAM 建图** | Cartographer（激光+编码器，无IMU建图） |
| **自适应定位** | AMCL + EKF（编码器+IMU+激光融合） |
| **路径规划** | SmacPlanner2D（MOORE 8方向搜索）+ RegulatedPurePursuit |
| **AI 语音导航** | LangGraph + DeepSeek API → 工具调用 → 机器人动作 |
| **Web 远程控制** | Flask + SocketIO + Qt 桌面客户端 |
| **自动巡逻/跟随** | LangGraph Agent → patrol_node.py（YOLOv8 视觉跟随） |
| **电池管理** | ADC 电压采集 + 低压语音播报 |

### 1.1 项目目录总结构

```
~/chapt9_update/
├── fishbot_ws/                         # ROS 2 Humble 工作空间
│   ├── src/
│   │   ├── autopatrol_robot/           # AI 智能体 + Web/Flask 控制 + 巡逻跟随
│   │   ├── custom_interfaces/          # 自定义 ROS2 服务定义 (Command.srv)
│   │   ├── fishbot_application/        # 导航应用层（导航到位、航点跟随、姿态初始化）
│   │   ├── fishbot_bringup/            # 启动总控 + IMU偏置校准 + odom2tf + 协方差注入
│   │   ├── fishbot_description/        # URDF 机器人模型描述
│   │   ├── fishbot_navigation2/        # Navigation2 配置 + EKF + 地图文件
│   │   ├── micro_ros_agent/            # micro-ROS UDP WiFi 代理 (8888端口)
│   │   ├── micro_ros_msgs/             # micro-ROS 消息定义 (Graph/Entity/Node)
│   │   ├── robot_controller_qt/        # Qt 桌面 GUI 控制程序
│   │   ├── ros_serial2wifi/            # 激光雷达串口→TCP 桥接
│   │   └── ydlidar_ros2/              # YDLIDAR 激光雷达驱动 (LifecycleNode)
│   ├── build/ install/ log/            # 编译产物
│   └── yolov8n.pt                      # YOLOv8 目标检测模型
│
├── fishbot_motion_control/             # PlatformIO ESP32 下位机固件
│   ├── src/main.cpp                    # 主控程序 (PID + 运动学 + micro-ROS + IMU)
│   ├── lib/Kinematics/                 # 差速运动学正逆解 + 里程计
│   ├── lib/PidController/              # PID 速度控制器
│   ├── lib/Button/                     # 物理按键检测
│   ├── lib/Servo/                      # 舵机控制
│   └── platformio.ini                  # PlatformIO 构建配置
│
├── build/                              # 旧构建目录（含 micro_ros_agent 编译）
├── DESIGN_MAPPING.md                   # Cartographer 建图功能设计方案
├── ODOMETRY_FUSION_DOC.md              # /odom 里程计全链路融合说明
└── SUMMARY_REPORT.md                   # 前期修复工作总结报告
```

### 1.2 技术栈一览

| 层次 | 技术 | 版本/说明 |
|------|------|----------|
| **ROS 层** | ROS 2 Humble | `/opt/ros/humble/` |
| **下位机** | ESP32 + micro-ROS | PlatformIO, Arduino 框架, UDP WiFi |
| **SLAM** | Cartographer ROS2 | `ros-humble-cartographer-ros` |
| **定位** | AMCL + EKF | `nav2_amcl` + `robot_localization` |
| **路径规划** | SmacPlanner2D | Nav2 Smac Hybrid-A* 2D |
| **路径跟踪** | RegulatedPurePursuit | Nav2 |
| **AI 框架** | LangGraph + LangChain + DeepSeek | API: `deepseek-chat` |
| **Web 服务** | Flask + SocketIO | 端口 5000 |
| **桌面 GUI** | Qt5 (C++) | robot_controller_qt |
| **YOLO** | YOLOv8n | 目标检测 / 跟随模式 |
| **激光雷达** | YDLIDAR | 通过 ros_serial2wifi TCP 桥接 |
| **IMU** | MPU6050 | I2C, 陀螺Z轴偏置自动校准 |

---

## 2. 硬件架构

### 2.1 硬件清单

| 组件 | 型号/参数 | 接口 |
|------|----------|------|
| **主控** | ESP32 | WiFi UDP → micro-ROS Agent |
| **电机驱动** | McpwmMotor | GPIO 22/23 (左), 12/13 (右) |
| **编码器** | PCNT 脉冲计数 | GPIO 32/33 (左), 26/25 (右) |
| **IMU** | MPU6050 | I2C (SDA=18, SCL=19) |
| **激光雷达** | YDLIDAR | 串口 → ros_serial2wifi → `/tmp/tty_laser` |
| **语音播报** | DFPlayer Mini | Serial2 (RX/GPIO16, TX/GPIO17) |
| **物理按键** | GPIO 15 | 用于舵机播报交互 |
| **电池检测** | ADC GPIO 34 | 分压比 5.17, 3S 锂电池 |

### 2.2 运动学参数

| 参数 | 值 | 来源 |
|------|-----|------|
| 轮距 | 212.3 mm | `kinematics.set_wheel_distance(212.3)` |
| 左轮 PID | [0.625, 0.125, 0.0] | `pid_controller[0].update_pid()` |
| 右轮 PID | [0.710, 0.125, 0.0] | `pid_controller[1].update_pid()` |
| 编码器分辨率 | 0.10805 m/脉冲 | `kinematics.set_motor_param()` |
| 机器人半径 | 0.272 m | Nav2 local_costmap |
| 电池分压比 | 5.17 | 电压检测电路 |

### 2.3 GPIO 分配

```
左电机:  PWM=22, DIR=23
右电机:  PWM=12, DIR=13
左编码器: A=32, B=33
右编码器: A=26, B=25
按键:      15
电池ADC:   34
I2C:       SDA=18, SCL=19
DFPlayer:  Serial2 (RX=16, TX=17)
```

---

## 3. 软件包清单与功能

### 3.1 fishbot_motion_control（ESP32 固件）

| 文件 | 功能 |
|------|------|
| `src/main.cpp` | 主控程序：PID 速度控制、差速运动学、micro-ROS 通信、IMU 采集、电池监测、舵机播报 |
| `lib/Kinematics/` | 差速运动学正逆解 + 里程计积分（`update_motor_speed` → `get_odom`） |
| `lib/PidController/` | PID 控制器：`update(current)` → PWM 输出，`clear_error()` 清零积分 |
| `lib/Button/` | 按键消抖检测，支持长按/短按 |
| `lib/Servo/` | 舵机 PWM 控制 |

**ESP32 发布的话题**:

| 话题 | 类型 | QoS | 频率 | 内容 |
|------|------|-----|------|------|
| `/odom` | `nav_msgs/Odometry` | BEST_EFFORT | 50 Hz | 编码器里程计 (x, y, yaw, vx) |
| `/imu_raw` | `sensor_msgs/Imu` | BEST_EFFORT | 100 Hz | MPU6050 原始数据 (含陀螺偏置) |
| `/battery_voltage` | `std_msgs/Float32` | DEFAULT | 1/60 Hz | 电池电压 (V) |
| `/battery_percent` | `std_msgs/Float32` | DEFAULT | 1/60 Hz | 电量百分比 (%) |

**ESP32 订阅的话题**:

| 话题 | 类型 | QoS | 用途 |
|------|------|-----|------|
| `/cmd_vel` | `geometry_msgs/Twist` | BEST_EFFORT | 速度指令（Nav2 controller） |
| `/nav_node_status` | `std_msgs/Bool` | DEFAULT | 导航节点运行状态 |

**ESP32 提供的服务**:

| 服务 | 类型 | 用途 |
|------|------|------|
| `/esp32_command` | `std_srvs/SetBool` | `true`=重启, `false`=舵机播报动作 |

**关键逻辑**:
- **3 秒无指令自动停转**: `last_cmd_vel_time` 超时 3000ms → 清零 PID 目标 + 停转电机
- **导航节点未运行时停转**: `/nav_node_status=false` → 主动停转（防止手动遥控与导航冲突时的残余速度）
- **低电量语音播报**: 电压 ≤ 11.4V 时 DFPlayer 播放第7段音频，滞回 0.3V

---

### 3.2 fishbot_bringup（系统启动总控）

| 文件 | 语言 | 功能 |
|------|------|------|
| `launch/bringup.launch.py` | Python | **基础启动**：urdf2tf + odom2tf + odom_covariance + micro_ros_agent + ros_serial2wifi + ydlidar(延时5s) |
| `launch/cartographer_slam.launch.py` | Python | Cartographer SLAM 建图启动 |
| `launch/urdf2tf.launch.py` | Python | URDF 模型 → TF 广播 |
| `launch/ydlidar_cartographer.launch.py` | Python | YDLIDAR + Cartographer 整合启动（原始备份） |
| `src/odom2tf.cpp` | C++ | 订阅 `/odometry/filtered` → 广播 `odom→base_footprint` TF |
| `src/imu_bias_remover.cpp` | C++ | 静止时采样陀螺Z轴偏置，输出 `/imu_corrected` |
| `fishbot_bringup/odom_covariance_injector.py` | Python | `/odom` → 补协方差 → `/odom_ekf` |
| `fishbot_bringup/scan_qos_bridge.py` | Python | 激光扫描 QoS 桥接（BEST_EFFORT → RELIABLE） |
| `fishbot_bringup/save_map.py` | Python | 地图保存工具（封装 map_saver_cli） |
| `fishbot_bringup/calibrate_wheelbase.py` | Python | 轮距自动标定脚本 |
| `fishbot_bringup/monitor_ekf.py` | Python | EKF 状态监控脚本 |
| `fishbot_bringup/bag_analyzer.py` | Python | ROS2 Bag 分析工具 |
| `config/fishbot_cartographer.lua` | Lua | Cartographer SLAM 配置文件 |
| `config/cartographer_slam.rviz` | YAML | Cartographer 建图 RViz 配置 |

**bringup.launch.py 启动的节点**:

```
┌────────────────────────────────────────────────────────────┐
│                    bringup.launch.py                        │
│                                                             │
│  ① urdf2tf          — URDF → TF (base_footprint→base_link  │
│                                          →laser_link)      │
│  ② odom_covariance  — /odom → /odom_ekf (补协方差)         │
│  ③ odom2tf          — /odometry/filtered → TF广播          │
│  ④ micro_ros_agent  — UDP:8888 代理 (WiFi→ROS2桥)          │
│  ⑤ ros_serial2wifi  — /tmp/tty_laser TCP服务器             │
│  ⑥ ydlidar (延时5s) — YDLIDAR LifecycleNode               │
└────────────────────────────────────────────────────────────┘
```

---

### 3.3 fishbot_navigation2（导航与定位）

| 文件 | 功能 |
|------|------|
| `launch/navigation2.launch.py` | **导航启动**：imu_bias_remover + EKF + Nav2 完整 stack |
| `launch/ekf.launch.py` | 单独启动 EKF 节点（调试用） |
| `config/ekf.yaml` | EKF 融合配置：odom0(编码器:x,y,yaw,vx) + imu0(陀螺:wz) |
| `config/nav2_params.yaml` | Nav2 完整参数（AMCL + Planner + Controller + BT + Costmap） |
| `maps/*.pgm + *.yaml` | 多张地图：fishbot_map1, home, home2, home5, room3 |

**EKF 配置核心**:
```yaml
odom0: /odom_ekf              # 编码器（已注入协方差）
odom0_config:
  [true, true, false,         # x, y
   false, false, true,        #       yaw
   true,  false, false,       # vx
   false, false, false,       #
   false, false, false, false]#

imu0: /imu_corrected           # 修正后IMU（去偏置）
imu0_config:
  [false, false, false,        #
   false, false, false,        #
   false, false, false,        #
   false, false, true,         #           wz
   false, false, false, false] #
```

---

### 3.4 autopatrol_robot（AI 智能体 + Web 控制）

这是整个系统最复杂的包，拆分为 **三层架构**：

```
┌──────────────────────────────────────────────────────────────┐
│  Web 层 (Flask + SocketIO)      AI 层 (LangGraph + DeepSeek) │
│       │                                │                      │
│       └──────────┬─────────────────────┘                      │
│                  ▼                                            │
│   NavigationService ──── 统一导航服务（线程安全）              │
│   NavManager        ──── CLI 子进程导航管理器                  │
│   LocationStore     ──── patrol_config.yaml 地点存储           │
│                  │                                            │
│                  ▼                                            │
│   PatrolNode (BasicNavigator) / nav_to_pose CLI               │
│                  │                                            │
│                  ▼                                            │
│   Nav2 Stack (AMCL + SmacPlanner2D + RegulatedPurePursuit)    │
└──────────────────────────────────────────────────────────────┘
```

**核心 Python 模块**:

| 文件 | 功能 |
|------|------|
| `web_controller.py` | Flask Web 服务器 + SocketIO 双向通信，管理 Web 会话和 AI 对话 |
| `agent_graph.py` | LangGraph Agent 定义：DeepSeek LLM + 工具绑定 + InMemorySaver checkpoint |
| `navigation_service.py` | 统一导航服务层，Web UI 和 AI Agent 的唯一边界 |
| `nav_manager.py` | CLI 进程生命周期管理 + 全局导航互斥锁 |
| `location_store.py` | `patrol_config.yaml` 单文件地点 CRUD（fcntl 文件锁） |
| `patrol_node.py` | 巡逻/跟随模式基类（BasicNavigator + ServoClient） |
| `tools/__init__.py` | 导出 `ALL_TOOLS`、`cleanup_ros()` |
| `tools/nav_tools.py` | AI 导航工具：导航到地点/坐标/序列，取消导航 |
| `tools/loc_tools.py` | AI 地点工具：列出/保存/删除/清空地点 |
| `tools/sys_tools.py` | AI 系统工具：初始化姿态、获取位置、电量、航点列表 |
| `tools/follow_tools.py` | AI 跟随工具：启动/停止/查询看护模式 |

**AI Agent 可调用的 14 个工具**:

| 工具 | 功能 |
|------|------|
| `list_locations` | 列出所有可用地点 |
| `list_saved_locations` | 列出自定义地点 |
| `save_current_location` | 保存当前位置为自定义地点 |
| `delete_saved_location` | 删除指定自定义地点 |
| `clear_all_saved_locations` | 清空所有自定义地点 |
| `navigate_to_location` | 导航到预定义地点（起点/厨房/客厅/影音房） |
| `navigate_sequence` | 依次导航到多个地点 |
| `navigate_to_saved_location` | 导航到自定义地点 |
| `navigate_to_xy` | 导航到任意坐标 |
| `cancel_navigation` | 取消导航 |
| `return_to_start` | 返回起点 |
| `get_navigation_status` | 查询导航状态 |
| `initialize_pose` | 初始化 AMCL 定位 |
| `get_current_position` | 获取当前位置 |
| `get_battery_status` | 查询电量 |
| `list_predefined_points` | 列出预定义航点 |
| `start_patrol_mode` | 开启看护/跟随模式 |
| `stop_patrol_mode` | 关闭看护/跟随模式 |
| `get_patrol_mode_status` | 查询看护模式状态 |

**Web 控制功能**:

| 功能 | 实现 |
|------|------|
| 地图可视化 | 订阅 `/map` 渲染 OccupancyGrid |
| 机器人轨迹 | 订阅 `/amcl_pose` 实时绘制 |
| 手动方向控制 | 前后左右旋转 → `/cmd_vel` |
| AI 对话 | SocketIO → LangGraph Agent → 工具调用 |
| 导航执行 | Web UI → NavigationService → PatrolNode |
| 电池监控 | 订阅 `/battery_voltage` 实时显示 |
| 摄像头流 | CameraWidget (Qt 端) |

---

### 3.5 fishbot_application（导航应用层）

| 文件 | 功能 |
|------|------|
| `init_robot_pose.py` | 初始化 AMCL 初始位姿（设置起点） |
| `get_robot_pose.py` | 获取机器人当前位姿 |
| `nav_to_pose.py` | CLI 导航工具：`ros2 run fishbot_application nav_to_pose --x 1.0 --y 2.0 --yaw 0.0` |
| `waypoint_follower.py` | 航点顺序跟随：读取 `patrol_config.yaml` 依次导航 |

---

### 3.6 fishbot_description（URDF 模型）

**TF 树**:
```
base_footprint ──(fixed)──→ base_link ──(fixed)──→ laser_link
    z=0                     z=0.076                z=+0.075
```

- `base_footprint`: 地面投影点（原点）
- `base_link`: 底盘中心（圆柱体 r=0.10m, h=0.12m）
- `laser_link`: 激光雷达安装位（圆柱体 r=0.02m, h=0.02m）

---

### 3.7 robot_controller_qt（Qt 桌面 GUI）

| 文件 | 功能 |
|------|------|
| `src/main.cpp` | Qt 主窗口：连接管理、手动控制、导航队列、一键返航 |
| `src/manual_controller.cpp` | 手动方向控制（Python 桥接发送 `/cmd_vel`） |
| `src/ros_map_bridge.cpp` | ROS2 桥接：订阅 `/map`、`/amcl_pose`、发布 `/goal_pose` |
| `src/map_display_widget.cpp` | 地图渲染 + 机器人轨迹绘制 |
| `src/camera_widget.cpp` | 摄像头视频流显示 |
| `src/launch_manager.cpp` | ROS2 Launch 进程管理（bringup/navigation2/cartographer） |
| `robot_control_bridge.py` | Python 桥接脚本（处理 Qt→ROS2 通信） |

---

### 3.8 其他包

| 包 | 功能 |
|------|------|
| `micro_ros_agent` | micro-ROS UDP 代理，监听 8888 端口，桥接 ESP32 WiFi → ROS2 |
| `micro_ros_msgs` | micro-ROS 内部消息定义（Graph/Entity/Node） |
| `ros_serial2wifi` | TCP 服务器，将 `/tmp/tty_laser` 虚拟串口暴露为 TCP，供 ydlidar 驱动使用 |
| `ydlidar_ros2` | YDLIDAR 官方 ROS2 驱动（LifecycleNode），5 秒延时启动 |
| `custom_interfaces` | 自定义服务 `Command.srv` |

---

## 4. 全系统数据流

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              ESP32 (micro-ROS WiFi UDP)                           │
│                                                                                   │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐   │
│  │ PCNT编码器│───▶│ PID控制器│───▶│ 运动学    │───▶│ /odom    │    │ DFPlayer │   │
│  │(左右轮)  │    │(速度闭环)│    │(差速模型)│    │ 50Hz     │    │(语音播报)│   │
│  └──────────┘    └──────────┘    └──────────┘    └────┬─────┘    └────┬─────┘   │
│                                                       │               │          │
│  ┌──────────┐                                        │               │          │
│  │ MPU6050  │──▶ /imu_raw 100Hz                      │         ┌─────▼─────┐   │
│  │(陀螺+加) │                                        │         │ 按键中断   │   │
│  └──────────┘                                        │         │(舵机播报) │   │
│                                                       │         └───────────┘   │
│  ┌──────────┐                                        │                          │
│  │ ADC 电池 │──▶ /battery_voltage, /battery_percent   │                          │
│  └──────────┘                                        │                          │
└──────────────────────────────────────────────────────┼──────────────────────────┘
                                                       │ UDP WiFi
                                              ┌────────▼────────┐
                                              │ micro_ros_agent  │
                                              │ (UDP:8888)       │
                                              └────────┬────────┘
                                                       │ ROS2
        ┌──────────────────────────────────────────────┼──────────────────────────────┐
        │                                              ▼                              │
        │  ┌────────────────────┐    ┌─────────────────────┐                           │
        │  │ odom_covariance    │    │ imu_bias_remover    │                           │
        │  │ injector.py        │    │ (C++ 节点)           │                           │
        │  │                    │    │                      │                           │
        │  │ /odom → /odom_ekf  │    │ /imu_raw →           │                           │
        │  │ (注入合理协方差)    │    │ /imu_corrected       │                           │
        │  └────────┬───────────┘    │ (去陀螺Z轴偏置)      │                           │
        │           │                └──────────┬──────────┘                           │
        │           │                           │                                      │
        │           └──────────┬────────────────┘                                      │
        │                      ▼                                                       │
        │  ┌─────────────────────────────────────────────┐                             │
        │  │           EKF (robot_localization)           │                             │
        │  │                                              │                             │
        │  │  odom0: /odom_ekf   [x, y, yaw, vx]         │                             │
        │  │  imu0:  /imu_corrected [wz]                  │                             │
        │  │                                              │                             │
        │  │  输出: /odometry/filtered                    │                             │
        │  └────────────────────┬────────────────────────┘                             │
        │                       │                                                      │
        │                       ▼                                                      │
        │  ┌─────────────────────────────────────────────┐                             │
        │  │              odom2tf (C++)                    │                             │
        │  │  /odometry/filtered → TF: odom→base_footprint│                             │
        │  └────────────────────┬────────────────────────┘                             │
        │                       │                                                      │
        └───────────────────────┼──────────────────────────────────────────────────────┘
                                │
    ┌───────────────────────────┼───────────────────────────────────────────┐
    │                    Navigation2 Stack                                    │
    │                           │                                            │
    │  ┌────────────────────────▼──────────────────────────────┐            │
    │  │              AMCL (粒子滤波定位)                        │            │
    │  │  输入: /scan + TF(odom→base_footprint) + /map         │            │
    │  │  输出: /amcl_pose, TF(map→odom)                       │            │
    │  └────────────────────────┬──────────────────────────────┘            │
    │                           │                                            │
    │  ┌────────────────────────▼──────────────────────────────┐            │
    │  │    Planner (SmacPlanner2D) + Controller (RPP)          │            │
    │  │                                                        │            │
    │  │  输入: /goal_pose, /map, TF, /odom                    │            │
    │  │  输出: /cmd_vel → ESP32                                │            │
    │  └───────────────────────────────────────────────────────┘            │
    └────────────────────────────────────────────────────────────────────────┘
```

---

## 5. 核心链路深度解析

### 5.1 里程计链路（详见 `ODOMETRY_FUSION_DOC.md`）

```
ESP32 编码器 → /odom (50Hz, cov=0)
     │
     ▼
odom_covariance_injector.py  → /odom_ekf (补协方差)
     │
     ▼
EKF (odom0: x,y,yaw,vx + imu0: wz) → /odometry/filtered
     │
     ▼
odom2tf.cpp  → TF: odom→base_footprint
     │
     ▼
AMCL 消费 TF 做运动模型更新
Controller 消费 /odom 做速度反馈
```

### 5.2 IMU 链路

```
MPU6050 → /imu_raw (含陀螺偏置 ~0.005 rad/s)
     │
     ▼
imu_bias_remover.cpp:
  - 静止检测 (|vx|<0.01, |wz|<0.02)
  - 初始标定 100样本/3秒
  - 运行时重标定 50样本/1.5秒
  - 中位数估计偏置
     │
     ▼
/imu_corrected (wz - bias)
     │
     ▼
EKF (imu0: 只取 wz, 不取 yaw)
```

**为什么 IMU 不提供 yaw？** — MPU6050 的 `getAngleZ()` 通过纯陀螺积分计算，长期漂移（几分钟内可达几十度）。改为只提供短期精准的角速度 wz，让 EKF 从编码器获取稳定的绝对 yaw。

### 5.3 导航链路

```
目标设定 (Web UI / AI Agent / Qt GUI)
     │
     ▼
NavigationService.navigate_to() / NavManager CLI
     │
     ▼
/goal_pose → BehaviorTree → SmacPlanner2D → RegulatedPurePursuit
     │
     ▼
/cmd_vel → micro_ros_agent → ESP32 → PID → 电机
     │
     ▼
编码器 → /odom → EKF → /odometry/filtered → odom2tf → TF
     │
     ▼
AMCL 用 TF + /scan 更新粒子 → /amcl_pose
```

### 5.4 AI Agent 链路

```
用户输入 (Web SocketIO / 语音)
     │
     ▼
LangGraph Agent (agent_graph.py)
  - SystemMessage: 机器人控制助手角色
  - LLM: DeepSeek V4 (deepseek-chat, temperature=0.3)
  - 绑定 18 个工具（导航/地点/系统/跟随）
     │
     ▼
LLM 决策 → tool_call
     │
     ▼
ToolNode 执行 → NavManager CLI / LocationStore / 系统查询
     │
     ▼
结果返回 → LLM 生成自然语言回复 → SocketIO 推送给 Web UI
```

### 5.5 巡逻/跟随链路

```
start_patrol_mode() → subprocess: patrol_node.py
     │
     ├── YOLOv8 目标检测 → 计算目标位置 → /goal_pose
     ├── Nav2 导航 → ESP32 驱动
     └── 到达后 ServoClient.call_servo_action()
         ├── 停止机器人 (持续 1.5s 零速)
         ├── 注入零角速度 IMU
         ├── 发送 ESP32 舵机播报
         └── 等待按键确认
```

---

## 6. 修复与优化清单（43 项）

### 6.1 阶段一：P0 关键 Bug 修复（5 项）

| # | 文件 | 问题 | 修复 |
|:-:|------|------|------|
| 1 | `navigation2.launch.py` | EKF 节点创建后返回值被丢弃，未加入 LaunchDescription | 赋值变量并加入 LD |
| 2 | `main.cpp` (ESP32) | `rclc_executor_add_subscription` 传入 NULL 指针 | 创建正确类型变量并传递地址 |
| 3 | `patrol_node.py` | ServoClient 未初始化 `cmd_vel_pub` | 添加 publisher 和 logger 初始化 |
| 4 | `agent_tools.py` | `cleanup_ros()` 引用未定义的全局变量 | 添加 `global` 声明 |
| 5 | `nav_to_pose.py` | `quaternion.w = 3.14` 非法四元数 | 使用 `quaternion_from_euler(0,0,3.14)` |

### 6.2 阶段一：路径硬编码修复（7 项）

| # | 文件 | 修复方式 |
|:-:|------|---------|
| 6 | `navigation2.launch.py` | 改用 `os.path.join(package_dir, 'config', 'ekf.yaml')` |
| 7 | `agent_tools.py` | 改用 `os.environ.get('FISHBOT_WS', ...)` |
| 8 | `web_controller.py` (3处) | 同上 |
| 9 | `ekf.launch.py` | 改用 `get_package_share_directory()` |
| 10 | `ydlidar_cartographer.launch.py` | URDF 用 `get_package_share_directory()` |
| 11 | `robot_controller_qt/*.cpp` (9处) | 改用 `qEnvironmentVariable("FISHBOT_WS", ...)` |
| 12 | `manual_controller.cpp` | 同上 |

### 6.3 阶段一：逻辑/配置 Bug（4 项）

| # | 问题 | 修复 |
|:-:|------|------|
| 13 | `bringup.launch.py`: `ros_serail2wifi` 拼写错误 | → `ros_serial2wifi` |
| 14 | `ekf.yaml`: `odom0_differential: false` | → `true`（编码器为差分里程计） |
| 15 | `waypoint_follower.py`: 用 `Trigger` 服务，ESP32 实际是 `SetBool` | → 改为 `SetBool` + `/esp32_command` |
| 16 | `display_robot.launch.py`: 引用不存在文件 | → 修正 URDF 和 rviz 路径 |

### 6.4 阶段二：EKF 策略大重构（4 项）

| # | 变更 |
|:-:|------|
| 17 | 删除 `speaker.py`（依赖不存在的 `SpeachText` 接口） |
| 18 | `ekf.yaml`: odom0 增加 yaw(index 5)，imu0 只保留 wz(index 11) |
| 19 | **新建** `imu_bias_remover.cpp`: 静止时自动标定陀螺偏置 |
| 20 | `CMakeLists.txt`: 添加 `sensor_msgs` 依赖 + `imu_bias_remover` 编译目标 |
| 21 | `navigation2.launch.py`: 在 EKF 前启动 `imu_bias_remover` |

### 6.5 阶段三：nav2_params.yaml 全面优化（22 项）

| # | 参数 | 原值 | 新值 | 原因 |
|:-:|------|:----:|:----:|------|
| 22 | `use_sim_time` (15处) | `True` | `False` | 真实机器人用系统时钟 |
| 23 | `alpha1` (旋转→角度噪声) | 0.1 | **0.8** | 轮式里程计旋转不可靠 |
| 24 | `alpha2` (平移→角度噪声) | 0.1 | **0.5** | 平移引起角度偏差 |
| 25 | `alpha3` (平移→位置噪声) | 0.1 | **0.4** | 考虑打滑 |
| 26 | `alpha4` (旋转→位置噪声) | 0.1 | **1.0** | 旋转时位置噪声严重 |
| 27 | `alpha5` (平移噪声相关) | 0.1 | **0.2** | 平移噪声相关度 |
| 28 | `z_hit / z_rand` | 0.5/0.5 | **0.9/0.05** | 90% 信任激光数据 |
| 29 | `update_min_d/a` | 0.25m/0.2rad | **0.1m/0.1rad** | 更频繁激光修正 |
| 30 | `min_particles` | 250 | **500** | 更大搜索空间 |
| 31 | `recovery_alpha` | 0.0/0.0 | **0.1/0.001** | 启用粒子恢复 |
| 32 | `transform_tolerance` | 1.0s | **0.2s** | 减少 TF 延迟 |
| 33 | `laser_likelihood_max_dist` | 2.0m | **5.0m** | 利用 YDLIDAR 量程 |
| 34 | local_costmap 障碍物层 | VoxelLayer | **ObstacleLayer** | 减少 CPU |
| 35 | global_costmap 感知范围 | 2.5/3.0m | **4.0/5.0m** | 看得更远 |
| 36 | Planner 搜索模型 | VON_NEUMANN | **MOORE** | 8方向更平滑 |
| 37 | Planner 期望频率 | 20Hz | **5Hz** | 减少超时 |
| 38 | `goal_tolerance` | 0.25m/0.25rad | **0.15m/0.15rad** | 更精确 |
| 39 | `behavior spin_vel` | 0.8 (> max 0.6) | **0.6** | 参数一致 |
| 40 | `max_rotation_time` | 5.0s | **8.0s** | 足够 180° 旋转 |
| 41-43 | 死代码清理 + 备份 | — | 已清理 + 创建 `.bak` | 配置整洁 |

---

## 7. 启动流程

### 7.1 完整启动步骤

```bash
# 0. 设置环境变量
export FISHBOT_WS=/home/ke/chapt9_update/fishbot_ws
source $FISHBOT_WS/install/setup.bash

# 1. 启动底盘基础功能（常驻后台）
ros2 launch fishbot_bringup bringup.launch.py

# 2. 选择导航模式：

# 2a. 建图模式 (Cartographer SLAM)
ros2 launch fishbot_bringup cartographer_slam.launch.py
# 建图完成后保存:
ros2 run fishbot_bringup save_map --name my_map

# 2b. 定位导航模式 (AMCL + EKF + Nav2)
ros2 launch fishbot_navigation2 navigation2.launch.py \
    map:=$FISHBOT_WS/src/fishbot_navigation2/maps/home.yaml

# 3. 可选：启动 Web 控制界面
ros2 run autopatrol_robot web_controller
# 浏览器访问 http://<机器人IP>:5000

# 4. 可选：启动 Qt 桌面控制界面
ros2 run robot_controller_qt robot_controller_qt

# 5. 可选：CLI 导航
ros2 run fishbot_application nav_to_pose --x 1.0 --y 2.0 --yaw 0.0
```

### 7.2 启动时序

```
bringup.launch.py (先启动)
    │
    ├── urdf2tf (TF 广播)
    ├── odom_covariance_injector (/odom → /odom_ekf)
    ├── odom2tf (/odometry/filtered → TF)
    ├── micro_ros_agent (UDP:8888, ESP32 连接)
    ├── ros_serial2wifi (/tmp/tty_laser TCP)
    └── ydlidar (5秒后启动，等待硬件稳定)

navigation2.launch.py (后启动，依赖 bringup)
    │
    ├── imu_bias_remover (静止标定陀螺偏置)
    ├── ekf_node (融合编码器+IMU)
    └── nav2_bringup (AMCL + BT + Planner + Controller)
```

---

## 8. 关键设计决策

### 8.1 为什么 odom0_differential 要从 false 改为 true？

编码器里程计的 yaw 是**绝对朝向**（从运动学积分得到），不是增量。EKF 需要知道这一点才能正确融合。设为 `false` 时 EKF 错误地认为 odom 是差分输入，导致位置估计发散。

### 8.2 为什么 IMU 只提供 wz 不提供 yaw？

MPU6050 的 `getAngleZ()` 是纯陀螺积分，几分钟内漂移可达数十度。如果让 EKF 融合这个 yaw，会严重破坏编码器的稳定朝向。只取短期精准的角速度 wz，让 EKF 自己积分 yaw 并用编码器修正。

### 8.3 为什么需要 odom_covariance_injector？

ESP32 用 micro-ROS 发布 `/odom` 时协方差全为 0。EKF 读到零协方差会认为该传感器完美无误，导致自身协方差收敛到零，丧失融合能力。注入合理协方差 (pose_xy≈0.01, yaw≈0.005) 让 EKF 正确估计不确定性。

### 8.4 为什么需要 imu_bias_remover？

MPU6050 的陀螺有固定偏置（~0.005 rad/s），即使静止也会输出非零角速度。这个偏置会随温度变化（温漂）。imu_bias_remover 在机器人静止时自动标定当前偏置，用中位数估计（对偶尔的抖动/噪声不敏感）。

### 8.5 为什么需要 ServoClient 注入零角速度 IMU？

ESP32 舵机播报期间（最长 60 秒），`/imu_raw` 停止发布 → `imu_bias_remover` 停止 → EKF 进入纯预测模式。如果 EKF 最后一帧有残留角速度（陀螺偏置未完全消除），预测的外推 yaw 会不断漂移。注入 wz=0 的 IMU 消息确保预测期间 yaw 不漂。

### 8.6 为什么 NavManager 使用 stdout=DEVNULL？

CLI 子进程（如 `ros2 run fishbot_application nav_to_pose`）的高频日志会填满管道缓冲区（OS 默认 64KB），导致 `write()` 阻塞 → 子进程死锁。丢弃 stdout 避免此问题，日志查看 `~/.ros/log/`。

### 8.7 为什么全局导航互斥锁？

Web UI 和 AI Agent 可能同时发起导航请求。`_global_nav_lock` 确保同一时间只有一个导航进行，防止两个目标冲突。

---

## 9. 已知遗留问题

### 9.1 路径相关

| 问题 | 影响 | 建议 |
|------|------|------|
| `launch_manager.cpp` 仍有部分硬编码路径 | Qt GUI 移植到其他机器需修改 | 改用 `qEnvironmentVariable("FISHBOT_WS")` |
| `/home/ke/detection_ws` 在 `agent_tools.py` 中引用 | 跟随模式依赖外部视觉工作空间 | 改为环境变量或纳入本 workspace |
| `hardware_config.yaml` 中串口使用 `/dev/ttyACM0` | 不同设备可能不同 | 运行时通过参数覆盖 |

### 9.2 功能相关

| 问题 | 说明 |
|------|------|
| Cartographer 建图未集成到主流程 | 需单独 launch（`cartographer_slam.launch.py`），未在 `navigation2.launch.py` 中支持模式切换 |
| 无里程计自动标定 | `calibrate_wheelbase.py` 存在但未自动化，轮距和编码器分辨率需手动测量 |
| ESP32 故障恢复 | WiFi 断连后 micro-ROS 不会自动重连，需 ESP32 重启 |
| DFPlayer 语音内容 | 固定 7 段音频，内容不可动态修改 |

### 9.3 备份文件

| 文件 | 说明 |
|------|------|
| `nav2_params.yaml.bak` | Nav2 原始配置备份，可回滚 |
| `web_controller copy.py` | 副本，不影响功能 |
| `patrol_node（backup）.py` | 备份文件（中文全角括号文件名） |
| `ekf.yaml.last_backup` | EKF 配置备份 |
| `fishbot_map1.*.bak` | 地图备份 |

---

## 附录

### A. 完整话题列表

| 话题 | 类型 | 发布者 | 订阅者 |
|------|------|--------|--------|
| `/odom` | `Odometry` | ESP32 | odom_covariance_injector |
| `/odom_ekf` | `Odometry` | odom_covariance_injector | EKF, imu_bias_remover |
| `/imu_raw` | `Imu` | ESP32 | imu_bias_remover |
| `/imu_corrected` | `Imu` | imu_bias_remover | EKF (servo 期间也注入) |
| `/odometry/filtered` | `Odometry` | EKF | odom2tf, controller_server |
| `/scan` | `LaserScan` | ydlidar | AMCL, local_costmap |
| `/cmd_vel` | `Twist` | controller_server, manual_controller, patrol_node | ESP32 |
| `/map` | `OccupancyGrid` | map_server | AMCL, Web UI, Qt GUI |
| `/amcl_pose` | `PoseWithCovarianceStamped` | AMCL | Web UI, Qt GUI |
| `/goal_pose` | `PoseStamped` | NavigationService, nav_to_pose | BT Navigator |
| `/battery_voltage` | `Float32` | ESP32 | Web UI, AI tools |
| `/battery_percent` | `Float32` | ESP32 | Web UI, AI tools |
| `/nav_node_status` | `Bool` | navigation2.launch.py (event) | ESP32 |
| `/tf` | `TFMessage` | urdf2tf, odom2tf, AMCL | 所有 TF 消费者 |

### B. 完整 TF 树（导航模式）

```
map ──(AMCL)──→ odom ──(odom2tf)──→ base_footprint ──(urdf2tf)──→ base_link ──(urdf2tf)──→ laser_link
```

### C. 服务列表

| 服务 | 类型 | 服务端 | 客户端 |
|------|------|--------|--------|
| `/esp32_command` | `SetBool` | ESP32 | patrol_node (ServoClient) |
| `/reinitialize_global_localization` | `std_srvs/Empty` | AMCL | init_robot_pose |

### D. 关键配置文件位置

| 配置 | 路径 |
|------|------|
| EKF 融合 | `fishbot_navigation2/config/ekf.yaml` |
| Nav2 全套 | `fishbot_navigation2/config/nav2_params.yaml` |
| Cartographer | `fishbot_bringup/config/fishbot_cartographer.lua` |
| 巡逻航点 | `autopatrol_robot/config/patrol_config.yaml` |
| URDF 模型 | `fishbot_description/urdf/fishbot.urdf` |
| ESP32 固件 | `fishbot_motion_control/src/main.cpp` |

---

> **文档版本**: v2.0  
> **最后更新**: 2026-06-07  
> **对应工作空间**: `~/chapt9_update/`  
> **关联文档**: [`DESIGN_MAPPING.md`](DESIGN_MAPPING.md) · [`ODOMETRY_FUSION_DOC.md`](ODOMETRY_FUSION_DOC.md) · [`SUMMARY_REPORT.md`](SUMMARY_REPORT.md)
