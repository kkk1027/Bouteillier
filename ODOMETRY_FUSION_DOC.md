# FishBot /odom 里程计全链路融合说明文档

> **项目**: chapt9_update / fishbot_ws  
> **日期**: 2026-06-07  
> **涉及包**: fishbot_motion_control (ESP32), fishbot_bringup, fishbot_navigation2  

---

## 目录

1. [系统架构概览](#1-系统架构概览)
2. [数据流全景图](#2-数据流全景图)
3. [第一层：ESP32 原始里程计生成](#3-第一层esp32-原始里程计生成)
4. [第二层：协方差注入](#4-第二层协方差注入)
5. [第三层：IMU数据采集与陀螺偏置校准](#5-第三层imu数据采集与陀螺偏置校准)
6. [第四层：EKF传感器融合](#6-第四层ekf传感器融合)
7. [第五层：TF广播](#7-第五层tf广播)
8. [第六层：下游消费](#8-第六层下游消费)
9. [关键设计决策与历史Bug](#9-关键设计决策与历史bug)
10. [参数总览](#10-参数总览)
11. [启动顺序与依赖](#11-启动顺序与依赖)

---

## 1. 系统架构概览

FishBot 的里程计系统采用 **"ESP32硬件层 → 预处理层 → EKF融合层 → TF广播层 → 下游消费"** 五层架构：

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ESP32 (micro-ROS)                           │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                         │
│  │ PCNT编码器│  │ PID控制器│  │ 运动学模型│ ──►  /odom (50Hz)       │
│  └──────────┘  └──────────┘  └──────────┘                         │
│  ┌──────────┐                                                      │
│  │ MPU6050  │ ──►  /imu_raw                                        │
│  └──────────┘                                                      │
└─────────────────────────────────────────────────────────────────────┘
        │                                      │
        ▼                                      ▼
┌───────────────────────┐          ┌───────────────────────┐
│ odom_covariance_      │          │ imu_bias_remover      │
│ injector.py           │          │ (C++ 节点)             │
│                       │          │                       │
│ /odom → /odom_ekf     │          │ /imu_raw → /imu_      │
│ (补协方差)             │          │ corrected (去陀螺偏置) │
└───────────┬───────────┘          └───────────┬───────────┘
            │                                  │
            ▼                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     EKF (robot_localization)                        │
│                                                                     │
│  odom0: /odom_ekf    (编码器: x, y, yaw, vx)                      │
│  imu0:  /imu_corrected (陀螺: wz)                                  │
│                                                                     │
│  publish_tf: false     ← 禁止EKF广播TF（odom2tf负责）               │
│                                                                     │
│  输出: /odometry/filtered                                           │
└─────────────────────────────────┬───────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        odom2tf (C++ 节点)                            │
│                                                                     │
│  订阅 /odometry/filtered → 广播 odom→base_footprint TF             │
└─────────────────────────────────┬───────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                       Navigation2 Stack                             │
│  ┌──────────────────────────────────────────────────────┐          │
│  │ AMCL: TF (odom→base_footprint) 用于运动模型           │          │
│  │ Controller: /odom 话题 用于速度反馈                   │          │
│  │ Local Costmap: global_frame=odom                     │          │
│  │ Global Costmap: global_frame=map                     │          │
│  └──────────────────────────────────────────────────────┘          │
└─────────────────────────────────────────────────────────────────────┘
```

### 涉及的节点一览

| 序号 | 节点名 | 包 | 语言 | 功能 |
|------|--------|----| ----|------|
| ① | `pid_project` (ESP32) | — | C++ | 编码器+运动学 → `/odom`; MPU6050 → `/imu_raw` |
| ② | `odom_covariance_injector` | fishbot_bringup | Python | 给 `/odom` 补充协方差 → `/odom_ekf` |
| ③ | `imu_bias_remover` | fishbot_bringup | C++ | 去除陀螺Z轴偏置 → `/imu_corrected` |
| ④ | `ekf_filter_node` | robot_localization | C++ | 融合里程计+IMU → `/odometry/filtered` |
| ⑤ | `odom2tf` | fishbot_bringup | C++ | `/odometry/filtered` → TF广播 |
| ⑥ | AMCL | nav2_amcl | C++ | 粒子滤波定位，消费 TF |
| ⑦ | controller_server | nav2_controller | C++ | 路径跟踪，消费 `/odom` |

---

## 2. 数据流全景图

```
时间轴 (实箭头=数据流, 虚箭头=配置依赖)

ESP32 (micro-ROS over WiFi UDP, agent=192.168.2.128:8888)
│
├──► /odom              50Hz, BEST_EFFORT, frame_id="odom", child="base_footprint"
│    │                  cov全部为0 (ESP32 micro-ROS 不写协方差)
│    │
│    ▼
│    [odom_covariance_injector.py]      ── 注入合理协方差
│    │
│    ├──► /odom_ekf      50Hz, frame_id="odom", child="base_footprint"
│    │                    cov: pose(x=0.01,y=0.01,yaw=0.005), twist(vx=0.02,vyaw=0.01)
│    │
│    ▼
│    [EKF ekf_filter_node]  30Hz
│    │   odom0: /odom_ekf   (x, y, yaw, vx)
│    │   imu0:  /imu_corrected (wz)
│    │
│    ├──► /odometry/filtered  30Hz, 融合后里程计
│    │
│    ▼
│    [odom2tf.cpp]
│    │
│    └──► TF: odom → base_footprint  (跟随 /odometry/filtered 频率)
│
├──► /imu_raw             ~100Hz (由 publish_imu_data 在20ms timer内调用)
│    │   frame_id="imu_link", orientation来自mpu.getAngleZ()陀螺积分
│    │
│    ▼
│    [imu_bias_remover.cpp]         ── 同时订阅 /odom 检测静止状态
│    │
│    └──► /imu_corrected  ~100Hz, wz减去偏置(中位数), 其余透传
│         │
│         └──► EKF imu0 输入
│
├──► /battery_voltage    1/60 Hz (每60秒)
├──► /battery_percent    1/60 Hz
│
├──► /esp32_command      SetBool 服务 (启动/停止电机)
│
└──► 订阅 /cmd_vel       Twist, 最佳效果
     └──► 订阅 /nav_node_status  Bool (导航节点运行状态)
```

---

## 3. 第一层：ESP32 原始里程计生成

### 3.1 硬件平台

| 组件 | 型号/方式 | 用途 |
|------|-----------|------|
| MCU | ESP32 | 主控 |
| 电机驱动 | Esp32McpwmMotor | MCPWM输出，引脚: 左(22,23), 右(12,13) |
| 编码器 | Esp32PcntEncoder | PCNT脉冲计数，引脚: 左(32,33), 右(26,25) |
| IMU | MPU6050 | I2C (SDA=18, SCL=19) |
| 通信 | WiFi UDP micro-ROS | SSID="kkk", agent=192.168.2.128:8888 |

### 3.2 关键参数

```cpp
// 运动学参数 — main.cpp setup()
kinematics.set_wheel_distance(212.3);   // 轮距 (mm)
kinematics.set_motor_param(0, 0.10805); // 左轮: 每tick对应位移 (mm/tick)
kinematics.set_motor_param(1, 0.10805); // 右轮: 每tick对应位移 (mm/tick)

// PID参数
pid_controller[0].update_pid(0.625, 0.125, 0.0);  // 左轮 Kp, Ki, Kd
pid_controller[1].update_pid(0.710, 0.125, 0.0);  // 右轮 Kp, Ki, Kd
pid_controller[0].out_limit(-100, 100);            // 占空比限幅 ±100%
pid_controller[1].out_limit(-100, 100);
```

### 3.3 控制循环（loop函数，运行在核0）

```cpp
void loop() {
    // 1. 喂看门狗 (每5秒)
    
    // 2. 更新编码器读数 → 运动学正解 → 更新里程计位姿
    kinematics.update_motor_speed(millis(), 
        encoders[0].getTicks(), encoders[1].getTicks());
    
    // 3. 闲时停电机逻辑
    bool no_twist = (millis() - last_cmd_vel_time >= 3000);
    if (motor_enabled && no_twist && !nav_node_running) {
        // 停止电机
    }
    
    // 4. PID计算 + 电机输出
    if (motor_enabled) {
        pid_controller[0].update(encoders[0].getTicks());
        pid_controller[1].update(encoders[1].getTicks());
        motor.updateMotorSpeed(0, pid_controller[0].out);
        motor.updateMotorSpeed(1, pid_controller[1].out);
    }
}
```

**速度指令接收** (`twist_callback`, 在micro-ROS executor中回调):

```cpp
void twist_callback(const void* msg_in) {
    last_cmd_vel_time = millis();
    target_linear_speed = msg->linear.x * 1000;   // m/s → mm/s
    target_angular_speed = msg->angular.z;          // rad/s
    
    // 逆运动学: 目标速度 → 左右轮速度
    kinematics.kinematics_inverse(target_linear_speed, target_angular_speed,
        &out_left_speed, &out_right_speed);
    
    // 更新PID目标
    pid_controller[0].update_target(out_left_speed);
    pid_controller[1].update_target(out_right_speed);
}
```

### 3.4 里程计发布（timer_callback, 20ms=50Hz, 运行在micro-ROS executor中）

```cpp
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    // 从运动学对象获取里程计
    odom_t odom = kinematics.get_odom();
    
    // 时间戳: micro-ROS agent同步
    int64_t stamp = rmw_uros_epoch_millis();
    msg_odom.header.stamp.sec  = stamp / 1000;
    msg_odom.header.stamp.nanosec = (stamp % 1000) * 1e6;
    
    // frame_id
    msg_odom.header.frame_id = "odom";
    msg_odom.child_frame_id = "base_footprint";
    
    // 位姿
    msg_odom.pose.pose.position.x = odom.x;
    msg_odom.pose.pose.position.y = odom.y;
    msg_odom.pose.pose.orientation.z = sin(odom.angle * 0.5);
    msg_odom.pose.pose.orientation.w = cos(odom.angle * 0.5);
    
    // 速度
    msg_odom.twist.twist.linear.x = odom.linear_speed;
    msg_odom.twist.twist.angular.z = odom.angular_speed;
    
    // 发布 (BEST_EFFORT QoS)
    rcl_publish(&pub_odom, &msg_odom, NULL);
    
    // 同时发布IMU数据
    publish_imu_data();
}
```

### 3.5 IMU原始数据发布（publish_imu_data, 由timer_callback调用）

```cpp
void publish_imu_data() {
    mpu.update();  // 更新MPU6050读数
    
    // orientation: 使用 mpu.getAngleZ() 陀螺积分值 (⚠️ 有长期漂移)
    float yaw = mpu.getAngleZ() * M_PI / 180.0;
    msg_imu.orientation.z = sin(yaw / 2.0);
    msg_imu.orientation.w = cos(yaw / 2.0);
    
    // 角速度: 陀螺原始输出 (deg/s → rad/s)
    msg_imu.angular_velocity.x = mpu.getGyroX() * M_PI / 180.0;
    msg_imu.angular_velocity.y = mpu.getGyroY() * M_PI / 180.0;
    msg_imu.angular_velocity.z = mpu.getGyroZ() * M_PI / 180.0;
    
    // 线加速度: 加速度计原始输出 (m/s²)
    msg_imu.linear_acceleration.x = mpu.getAccX();
    msg_imu.linear_acceleration.y = mpu.getAccY();
    msg_imu.linear_acceleration.z = mpu.getAccZ();
    
    rcl_publish(&pub_imu, &msg_imu, NULL);
}
```

### 3.6 ESP32输出话题汇总

| 话题 | QoS | 频率 | 说明 |
|------|-----|------|------|
| `/odom` | BEST_EFFORT | ~50Hz | 编码器里程计，协方差全0 |
| `/imu_raw` | BEST_EFFORT | ~50Hz | MPU6050原始数据，wz含偏置 |
| `/battery_voltage` | default | 1/60 Hz | 电池电压 (V) |
| `/battery_percent` | default | 1/60 Hz | 电量百分比 |
| `/esp32_command` | 服务 | — | SetBool 启停电机 |

---

## 4. 第二层：协方差注入

### 4.1 为什么需要协方差注入？

ESP32 使用 micro-ROS 发布 `/odom` 时，**协方差矩阵全为零**。如果直接将全零协方差消息送入 EKF，EKF 会将编码器视为"完美传感器"——这意味着：

1. EKF 自身的状态协方差会收敛到零（认为自己对状态估计完全确定）
2. EKF 不再信任后续任何传感器更新（包括 IMU）
3. IMU 的 wz 数据无法修正里程计，融合完全失效

### 4.2 节点: `odom_covariance_injector.py`

**文件**: `fishbot_bringup/fishbot_bringup/odom_covariance_injector.py`

```python
class OdomCovarianceInjector(Node):
    def __init__(self):
        # 可调协方差参数 (ros2 param 可动态修改)
        self.declare_parameter('pose_cov_xy', 0.01)      # x,y 位置方差 (m²)
        self.declare_parameter('pose_cov_yaw', 0.005)    # yaw 朝向方差 (rad²)
        self.declare_parameter('twist_cov_vx', 0.02)     # vx 线速度方差 (m²/s²)
        self.declare_parameter('twist_cov_vyaw', 0.01)   # vyaw 角速度方差 (rad²/s²)
        
        # QoS: ESP32用BEST_EFFORT，订阅端必须匹配
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Odometry, '/odom', self.callback, qos)
        self.pub = self.create_publisher(Odometry, '/odom_ekf', 10)
```

**数据变换**: `/odom` (cov=0) → 注入协方差 → `/odom_ekf` (cov=合理值)

**注入的协方差矩阵结构** (6×6, 展开为36元素一维数组):
```
pose.covariance:
  [0]  = 0.01   ← x 方差 (m²)
  [7]  = 0.01   ← y 方差 (m²)
  [14] = 1e-9   ← z (2D模式不用但需>0)
  [21] = 1e-9   ← roll
  [28] = 1e-9   ← pitch
  [35] = 0.005  ← yaw 方差 (rad²)

twist.covariance:
  [0]  = 0.02   ← vx 方差 (m²/s²)
  [7]  = 1e-9   ← vy
  [14] = 1e-9   ← vz
  [21] = 1e-9   ← vroll
  [28] = 1e-9   ← vpitch
  [35] = 0.01   ← vyaw 方差 (rad²/s²)
```

**参数含义**:
- `pose_cov_xy=0.01`: 编码器推算的位置不确定性 ~10cm (1σ)，符合编码器打滑/轮径误差的实际情况
- `pose_cov_yaw=0.005`: 朝向不确定性 ~0.07 rad (~4°)，编码器对yaw的推算精度
- `twist_cov_vx=0.02`: 速度不确定性 ~0.14 m/s
- `twist_cov_vyaw=0.01`: 角速度不确定性 ~0.1 rad/s

> **调参提示**: 协方差越大 = EKF 越不信任该传感器 = EKF 越依赖自身运动模型预测  
> 协方差越小 = EKF 越信任该传感器 = EKF 越直接用传感器值修正状态

---

## 5. 第三层：IMU数据采集与陀螺偏置校准

### 5.1 MPU6050 陀螺偏置问题

MPU6050 是消费级MEMS陀螺仪，其Z轴陀螺在静止时通常有 **0.5~2.0°/s** 的零偏（bias）。如果直接将原始 wz 送入 EKF：

- EKF 认为机器人在持续旋转
- 编码器提供正确的绝对yaw（来自运动学），但会被带偏置的 wz "修正"
- 最终融合后的 yaw 会漂移

### 5.2 节点: `imu_bias_remover.cpp`

**文件**: `fishbot_bringup/src/imu_bias_remover.cpp`

#### 工作原理

```
┌─────────────────────────────────────────────────────────┐
│                    imu_bias_remover                      │
│                                                         │
│  输入:                                                   │
│    /imu_raw    — IMU原始数据                             │
│    /odom       — 编码器里程计 (用于检测静止)              │
│                                                         │
│  静止检测:                                               │
│    条件: |vx| < 0.01 m/s && |wz| < 0.02 rad/s           │
│                                                         │
│  偏置估算:                                               │
│    初始标定: 静止时采集 ≥100个样本, 持续≥3秒              │
│    运行时重标定: 静止时采集 ≥50个样本, 持续≥1.5秒         │
│    算法: 中位数 (median) — 抗MPU6050偶发尖峰              │
│                                                         │
│  输出:                                                   │
│    /imu_corrected — wz = raw_wz - wz_bias               │
│    其他字段 (orientation, acceleration) 全部透传         │
└─────────────────────────────────────────────────────────┘
```

#### 可配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `stationary_vx_thresh` | 0.01 m/s | 静止线速度阈值 |
| `stationary_wz_thresh` | 0.02 rad/s | 静止角速度阈值 |
| `calib_samples` | 100 | 初始标定所需最少样本数 |
| `calib_duration` | 3.0 s | 初始标定所需最小时长 |

#### 两阶段偏置校准

1. **初始标定** (`bias_initialized_=false`):
   - 机器人首次静止时触发
   - 需要 ≥100 个样本 且 静止 ≥3 秒
   - 计算中位数偏置 → `wz_bias_`
   - 设置 `bias_initialized_=true`

2. **运行时重标定** (`bias_initialized_=true`):
   - MPU6050 偏置会随温度漂移
   - 每次静止 ≥1.5 秒 且 ≥50 个样本时更新偏置
   - 使用中位数避免离群值

#### 为什么使用中位数而非均值？

MPU6050 偶发会产生大幅尖峰（几百°/s的异常读数），均值会被尖峰拉偏，中位数不受影响。这在嘈杂的硬件总线环境（ESP32 WiFi + 电机PWM干扰）中至关重要。

---

## 6. 第四层：EKF传感器融合

### 6.1 融合策略概述

EKF (Extended Kalman Filter) 使用 `robot_localization` 包的 `ekf_node`，融合两个传感器源：

| 输入通道 | 话题 | 使用的维度 | 角色 |
|---------|------|-----------|------|
| odom0 | `/odom_ekf` | x, y (位置), yaw (朝向), vx (线速度) | **绝对参考** — 编码器推算，长期不漂移 |
| imu0 | `/imu_corrected` | wz (Z轴角速度) | **瞬时修正** — 陀螺短期精准，使旋转更平滑 |

### 6.2 配置文件: `ekf.yaml`

**文件**: `fishbot_navigation2/config/ekf.yaml`

```yaml
ekf_filter_node:
  ros__parameters:
    # ========== 基本设置 ==========
    frequency: 30.0                # EKF更新频率 (Hz)
    sensor_timeout: 0.5            # 传感器超时 (秒)
    two_d_mode: true               # 2D模式 (忽略z, roll, pitch)
    
    # ========== 坐标帧 ==========
    map_frame: map                 # 地图帧 (AMCL提供map→odom)
    odom_frame: odom               # 里程计帧 (EKF输出)
    base_link_frame: base_footprint # 机器人基座帧
    world_frame: odom              # EKF在世界坐标系odom中估计
    
    # ========== ⚠️ 关键：禁止EKF广播TF ==========
    publish_tf: false
    # 理由：odom2tf 节点专门负责发布 odom→base_footprint TF
    # 如果EKF也发布，会导致TF冲突（同一父子帧有两个发布者）
```

### 6.3 关键配置决策

#### 6.3.1 `publish_tf: false` — 为什么禁止EKF广播TF？

```
❌ 错误方式 (EKF直接广播TF):
   EKF ──TF(odom→base_footprint)──► TF树
   
   问题：
   1. EKF发布TF的时机由其30Hz内部循环决定
   2. 当AMCL发布 map→odom 矫正时，EKF的TF可能与AMCL的TF时序冲突
   3. TF系统中同一父子帧不能有两个权威发布者

✅ 正确方式 (专用odom2tf节点):
   /odometry/filtered → odom2tf → TF(odom→base_footprint)
   
   优势：
   1. odom2tf 以最快的数据到达频率广播TF
   2. AMCL的 map→odom 和 odom2tf 的 odom→base_footprint 时序自然协调
```

#### 6.3.2 `odom0_differential: false` — 绝对模式

- `false`: EKF将 `/odom_ekf` 中的 x, y, yaw 视为**绝对位姿估计**（相对于 odom 原点）
- `true`: EKF将数据视为**增量**（两帧之间的变化量）

编码器里程计从原点开始累积，提供的是**绝对位姿**，所以设为 `false`。

> **历史修复**: 之前在 memory 中记录的修复项 #14 将 `odom0_differential` 从 `false` 改为 `true`，但那是在不同上下文中的临时方案。当前最终配置为 `false`（绝对模式），因为编码器里程计确实提供绝对位姿。

#### 6.3.3 传感器维度配置 (`odom0_config`)

```
索引:  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14
含义:  x   y   z  rx  ry  rz  vx  vy  vz vrx vry vrz  ax  ay  az

odom0_config: [true, true, false, false, false, true,   ← 位姿: x, y, yaw
               true, false, false, false, false, false,  ← 速度: vx
               false, false, false]                       ← 加速度: 无

imu0_config:  [false, false, false, false, false, false, ← 位姿: 无
               false, false, false, false, false, true,   ← 速度: wz
               false, false, false]                       ← 加速度: 无
```

**odom0 使用的4个维度**:
| 索引 | 含义 | 启用 | 来源 |
|------|------|------|------|
| 0 | x | ✅ | 编码器正运动学 |
| 1 | y | ✅ | 编码器正运动学 |
| 5 | yaw | ✅ | 编码器差速模型 |
| 6 | vx | ✅ | 编码器微分 |

**imu0 使用的1个维度**:
| 索引 | 含义 | 启用 | 来源 |
|------|------|------|------|
| 11 | wz (angular.z) | ✅ | MPU6050陀螺 (已去偏置) |

### 6.4 融合原理

```
每一步EKF更新 (30Hz):

1. 预测步骤 (Process Model):
   基于上一帧状态 + 控制输入 (vx, wz) + 过程噪声协方差
   → 预测当前帧的 (x, y, yaw, vx)

2. 更新步骤 (Measurement Update):

   当收到 /odom_ekf 消息:
     用编码器的 (x, y, yaw, vx) 修正预测值
     权重 = 编码器协方差 vs 预测协方差
     → 编码器协方差大 → EKF更信任自己预测
     → 编码器协方差小 → EKF更信任编码器

   当收到 /imu_corrected 消息:
     用陀螺的 wz 修正预测的旋转
     权重 = IMU协方差 vs 预测协方差
     → IMU提供瞬时角速度，使旋转更平滑

3. 发布 /odometry/filtered:
   包含融合后的 (x, y, yaw, vx, wz) + 完整的6×6协方差矩阵
```

### 6.5 为什么"编码器保持yaw不漂移 + IMU使旋转更平滑"？

这是核心设计思路：

| 特性 | 编码器 yaw | IMU wz |
|------|-----------|--------|
| 精度 | 中等 (受轮径误差/打滑影响) | 高 (陀螺短期精度好) |
| 长期漂移 | ❌ 不漂移 (基于运动学，无积分误差) | ✅ 严重漂移 (积分累积) |
| 瞬时噪声 | 较高 (齿轮间隙、地面颠簸) | ✅ 低 |
| 对旋转的响应 | 有延迟 (需要轮子转动一定角度) | ✅ 即时 |

**EKF的效果**:
- 编码器提供 **绝对yaw锚定** — 确保长期yaw不漂移
- IMU wz 提供 **瞬时角速度平滑** — 减少编码器瞬时噪声对旋转估计的影响
- 两者互补，输出比单独使用任何一个都更稳定准确的位姿

### 6.6 `world_frame: odom` vs `map_frame: map`

```
world_frame: odom      ← EKF 在 odom 坐标系下估计机器人位姿
map_frame: map         ← 声明存在 map 坐标系（由 AMCL 提供 map→odom）

TF树结构:
  map ──(AMCL发布, 低频矫正)──► odom ──(odom2tf发布, 高频)──► base_footprint
                                                                    │
                                                         (URDF静态) ▼
                                                                    base_link
                                                                    │
                                                         (URDF静态) ▼
                                                                   laser_link
```

EKF 工作在 `odom` 坐标系下，AMCL 负责在 `map` 坐标系下通过 `map→odom` 的TF变换来矫正全局定位误差。

---

## 7. 第五层：TF广播

### 7.1 节点: `odom2tf.cpp`

**文件**: `fishbot_bringup/src/odom2tf.cpp`

这是一个非常简洁的"话题→TF"桥接节点：

```cpp
class OdomTopic2TF : public rclcpp::Node {
public:
  OdomTopic2TF(std::string name) : Node(name) {
    // 订阅 EKF 融合后的里程计
    odom_subscribe_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", rclcpp::SensorDataQoS(),
        std::bind(&OdomTopic2TF::odom_callback_, this, std::placeholders::_1));
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg->header;              // frame_id="odom", stamp
    transform.child_frame_id = msg->child_frame_id; // "base_footprint"
    
    // 直接复制位姿
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.rotation = msg->pose.pose.orientation;
    
    tf_broadcaster_->sendTransform(transform);
  }
};
```

### 7.2 为什么 odom2tf 订阅 `/odometry/filtered` 而不是 `/odom`？

**这是修复 #18 的核心**（见第9节历史Bug记录）。

| 方案 | 订阅话题 | 后果 |
|------|---------|------|
| ❌ 旧方案 | `/odom` (ESP32) | TF使用原始编码器yaw（漂移），点云跟随机器人旋转 |
| ✅ 新方案 | `/odometry/filtered` (EKF) | TF使用EKF融合后的yaw（编码器锚定+IMU平滑），点云稳定 |

原始编码器的yaw虽然绝对值不漂移，但在旋转时有较大瞬时误差（编码器离散性、PID跟踪延迟），导致TF在短暂时间内指向错误方向 → 激光点云看起来在旋转。EKF融合IMU wz后，瞬时yaw更平滑 → TF更稳定 → 点云不再跟随旋转。

---

## 8. 第六层：下游消费

### 8.1 Navigation2 Stack 如何消费里程计

Navigation2 中不同组件以不同方式消费里程计数据：

#### AMCL (自适应蒙特卡洛定位)
```
消费方式: TF (odom → base_footprint)
用途: 运动模型更新 (Motion Model)
原理: 两次激光扫描之间的机器人位移 = TF的变化量
      将TF变化量应用到每个粒子上 → 粒子扩散 → 激光匹配 → 重采样
```

#### Controller Server (RegulatedPurePursuit)
```
消费方式: /odom 话题 (Odometry消息)
用途: 速度反馈 (Velocity Feedback)
原理: 读取机器人当前速度 vx, wz → 与期望速度比较 → 调整输出 cmd_vel
配置: odom_topic: "odom"  (在 nav2_params.yaml 第378行)
```

#### Local Costmap
```
消费方式: TF (odom → base_footprint)
用途: 将机器人足迹转换到 odom 坐标系
配置: global_frame: odom, robot_base_frame: base_link
```

#### Global Costmap
```
消费方式: TF (map → base_footprint)
用途: 将机器人足迹转换到 map 坐标系
配置: global_frame: map, robot_base_frame: base_link
```

### 8.2 Controller 使用 `/odom` 而非 `/odometry/filtered` 的考虑

当前 nav2_params.yaml 中 controller 的 `odom_topic: "odom"`（指向ESP32原始 `/odom`），而非 EKF 的 `/odometry/filtered`。

**影响分析**:
- Controller 仅使用 `twist.twist`（vx, wz）做速度反馈
- 原始编码器速度 (vx) 精度足够（轮速直接测量）
- EKF 对速度的改善主要在 yaw（朝向），不影响 vx 精度
- 因此 controller 使用 `/odom` 是合理且更简单的选择

> 如果未来发现速度反馈抖动过大，可将 `odom_topic` 改为 `/odometry/filtered`，但从当前系统表现看无此必要。

---

## 9. 关键设计决策与历史Bug

### 9.1 历史修复记录

| 修复编号 | 问题 | 根因 | 解决方案 |
|----------|------|------|----------|
| #14 | EKF融合质量差 | `odom0_differential: false` 不正确 (当时误判) | 经过后续分析和测试，确认 `false`（绝对模式）是正确的（编码器提供绝对位姿） |
| #18 | 点云跟随机器人旋转 | `odom2tf` 订阅 `/odom`（原始编码器），瞬时yaw误差大 → TF抖动 → 点云旋转 | `odom2tf` 改为订阅 `/odometry/filtered`（EKF融合后yaw更平滑） |

### 9.2 修复 #18 详细复盘（最重要的架构修正）

**错误方案（已回退）**:
```
第一次尝试: 在 EKF launch 中添加 remapping:
    remappings=[('odometry/filtered', 'odom')]
    
问题: ekf.yaml 中 odom0: /odom_ekf
     EKF订阅 /odom_ekf 作为输入
     但 remapping 把输出 /odometry/filtered 映射到 /odom
    
     → EKF输入 odom0 订阅 /odom_ekf（没问题）
     → EKF输出 /odometry/filtered 被remap为 /odom
     → /odom 被 odom_covariance_injector 订阅 → 转发到 /odom_ekf
     → /odom_ekf 又被 EKF 订阅...
     
     ⚠️ 反馈环路！EKF 读到自己的输出作为输入 → 协方差发散 → TF爆炸
```

**正确方案（当前代码）**:
```
1. EKF launch: 不添加任何 remapping
2. EKF 输出: /odometry/filtered
3. odom2tf: 订阅 /odometry/filtered → 广播 TF
4. 数据流:
   ESP32 → /odom → cov_injector → /odom_ekf → EKF → /odometry/filtered → odom2tf → TF
```

### 9.3 为什么不在EKF输出端直接remap？

```
robot_localization 的 ekf_node 内部架构:

  ┌──────────────────────────────────────────┐
  │              ekf_filter_node              │
  │                                           │
  │  odom0 → /odom_ekf ──┐                   │
  │  imu0  → /imu_corrected ─┤                │
  │                           ▼               │
  │                    ┌──────────┐           │
  │                    │ EKF Core │           │
  │                    └────┬─────┘           │
  │                         │                 │
  │                    ┌────▼─────┐           │
  │                    │ Publisher │──► /odometry/filtered
  │                    └──────────┘           │
  │                                           │
  │  (if publish_tf=true) ──► TF odom→base    │
  └──────────────────────────────────────────┘

如果 remap /odometry/filtered → /odom:
  问题1: /odom 是公共话题，多个节点发布/订阅 → 混乱
  问题2: 如果哪个下游节点把 /odom 又转回给 EKF 输入 → 反馈环路
```

因此正确做法是保持 EKF 输出为独立话题 `/odometry/filtered`，让 odom2tf 专门做话题→TF 的桥接。

---

## 10. 参数总览

### 10.1 编码器 & 运动学参数 (ESP32 main.cpp)

| 参数 | 值 | 说明 |
|------|-----|------|
| wheel_distance | 212.3 mm | 左右轮间距 |
| motor_param[0] | 0.10805 mm/tick | 左轮：每编码器tick对应位移 |
| motor_param[1] | 0.10805 mm/tick | 右轮：每编码器tick对应位移 |
| PID左 Kp/Ki/Kd | 0.625 / 0.125 / 0.0 | 左轮PID |
| PID右 Kp/Ki/Kd | 0.710 / 0.125 / 0.0 | 右轮PID |
| 输出限幅 | ±100% | MCPWM占空比 |
| 里程计发布频率 | 50Hz (20ms timer) | |
| 闲时停电机超时 | 3000ms | 无cmd_vel指令后自动停转 |

### 10.2 协方差注入参数 (odom_covariance_injector.py)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| pose_cov_xy | 0.01 | x,y 位置不确定性 (m²), ~10cm std |
| pose_cov_yaw | 0.005 | yaw 不确定性 (rad²), ~4° std |
| twist_cov_vx | 0.02 | vx 速度不确定性 (m²/s²) |
| twist_cov_vyaw | 0.01 | vyaw 角速度不确定性 (rad²/s²) |

### 10.3 IMU偏置校准参数 (imu_bias_remover.cpp)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| stationary_vx_thresh | 0.01 m/s | 静止线速度判断阈值 |
| stationary_wz_thresh | 0.02 rad/s | 静止角速度判断阈值 |
| calib_samples | 100 | 初始标定样本数 |
| calib_duration | 3.0 s | 初始标定时长 |
| 重标定样本 | calib_samples/2 = 50 | |
| 重标定时长 | calib_duration/2 = 1.5s | |
| 统计算法 | 中位数 (median) | 抗MPU6050尖峰 |

### 10.4 EKF参数 (ekf.yaml)

| 参数 | 值 | 说明 |
|------|-----|------|
| frequency | 30.0 Hz | EKF更新频率 |
| sensor_timeout | 0.5 s | 传感器数据超时 |
| two_d_mode | true | 2D模式 |
| publish_tf | **false** | ⚠️ 禁止EKF广播TF |
| world_frame | odom | EKF估计坐标系 |
| odom_frame | odom | 输出frame_id |
| base_link_frame | base_footprint | 机器人基座帧 |
| odom0 | /odom_ekf | 编码器里程计输入 |
| odom0_differential | false | 绝对位姿模式 |
| odom0_relative | false | |
| odom0_queue_size | 10 | 传感器缓冲 |
| odom0_config | [x,y,_,_,_,yaw, vx,...] | 使用 x,y,yaw,vx |
| imu0 | /imu_corrected | IMU输入 |
| imu0_config | [_,_,_,_,_,_, _,_,_,_,_,wz] | 仅使用 wz |
| imu0_queue_size | 10 | |
| process_noise_covariance | (15×15对角) | 过程噪声: x,y=0.02, yaw=0.01, vx=0.02, vwz=0.05 |
| initial_estimate_covariance | (15×15对角) | 初始协方差: 全部1e-9 (从0开始) |

### 10.5 Navigation2 里程计相关参数 (nav2_params.yaml)

| 参数 | 值 | 所属节点 |
|------|-----|----------|
| odom_frame_id | "odom" | AMCL |
| odom_topic | /odom | bt_navigator |
| odom_topic | "odom" | controller_server |
| odom_duration | 0.1 | controller_server |
| local_costmap global_frame | odom | local_costmap |
| global_costmap global_frame | map | global_costmap |
| robot_base_frame | base_link | costmaps |

---

## 11. 启动顺序与依赖

### 11.1 启动文件层级

```
navigation2.launch.py (导航完整启动)
├── urdf2tf.launch.py         ① 发布 base_footprint→base_link→laser_link 静态TF
├── odom2tf                   ② 订阅 /odometry/filtered → TF(odom→base_footprint)
├── imu_bias_remover          ③ 订阅 /imu_raw + /odom → /imu_corrected
├── odom_covariance_injector  ④ 订阅 /odom → /odom_ekf
├── ekf_node                  ⑤ 订阅 /odom_ekf + /imu_corrected → /odometry/filtered
├── nav2_bringup_launch       ⑥ AMCL, Controller, Planner, Costmaps
└── rviz2                     ⑦ 可视化
```

### 11.2 独立启动 (bringup.launch.py)

```
bringup.launch.py (仅硬件驱动，不含导航)
├── urdf2tf                   ① 静态TF
├── odom_covariance_injector  ② /odom → /odom_ekf
├── odom2tf                   ③ /odometry/filtered → TF
├── micro_ros_agent           ④ UDP agent (端口8888)
├── ros_serial2wifi           ⑤ TCP→串口 (激光雷达桥接)
└── ydlidar (延迟5秒)         ⑥ 激光雷达驱动
```

### 11.3 依赖关系

```
                    ┌──────────────────┐
                    │   micro_ros_agent │ ← 必须先启动，ESP32连接依赖
                    └────────┬─────────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
        /odom (ESP32)  /imu_raw (ESP32)  /cmd_vel
              │              │
              ▼              ▼
    odom_covariance    imu_bias_remover
    _injector              │
         │                 ▼
         ▼          /imu_corrected
     /odom_ekf           │
         │               │
         └───────┬───────┘
                 ▼
              EKF (ekf_node)
                 │
                 ▼
         /odometry/filtered
                 │
                 ▼
              odom2tf
                 │
                 ▼
          TF: odom→base_footprint
                 │
        ┌────────┼────────┐
        ▼        ▼        ▼
      AMCL   Costmap  Controller
```

### 11.4 启动检查清单

```bash
# 1. 检查话题是否正常发布
ros2 topic hz /odom              # 应该 ~50Hz
ros2 topic hz /odom_ekf          # 应该 ~50Hz (协方差注入)
ros2 topic hz /imu_raw           # 应该 ~50Hz
ros2 topic hz /imu_corrected     # 应该 ~50Hz
ros2 topic hz /odometry/filtered # 应该 ~30Hz

# 2. 检查TF树
ros2 run tf2_tools view_frames
# 应看到: map → odom → base_footprint → base_link → laser_link

# 3. 检查EKF是否正确融合
ros2 topic echo /odometry/filtered --field pose.pose
# 观察 x, y, orientation.z/w 是否平滑变化

# 4. 检查协方差是否注入成功
ros2 topic echo /odom_ekf --field pose.covariance
# pose.covariance[0] 应该 ~0.01 (不是0)

# 5. 检查IMU偏置校准
ros2 run rqt_console rqt_console
# 寻找 "陀螺偏置初始标定完成" 日志
```

---

## 附录A: 话题与帧速查表

| 话题 | 消息类型 | 发布者 | 订阅者 | QoS |
|------|---------|--------|--------|-----|
| `/odom` | Odometry | ESP32 (pid_project) | odom_covariance_injector, imu_bias_remover, controller_server | BEST_EFFORT |
| `/odom_ekf` | Odometry | odom_covariance_injector | EKF (ekf_filter_node) | default |
| `/imu_raw` | Imu | ESP32 (pid_project) | imu_bias_remover | BEST_EFFORT |
| `/imu_corrected` | Imu | imu_bias_remover | EKF (ekf_filter_node) | default |
| `/odometry/filtered` | Odometry | EKF (ekf_filter_node) | odom2tf | default |
| `/cmd_vel` | Twist | controller_server / teleop | ESP32 (pid_project) | default→BEST_EFFORT |
| `/nav_node_status` | Bool | navigation_service / patrol_node | ESP32 (pid_project) | default |

## 附录B: TF帧关系

```
map                  ← AMCL 发布 (map→odom, 低频矫正, ~10Hz)
 │
 ├─ odom             ← EKF 输出 frame_id; TF 由 odom2tf 广播
 │    │
 │    ├─ base_footprint ← 机器人投影点 (无高度), TF 由 odom2tf 广播
 │         │
 │         ├─ base_link       ← 机器人本体, TF 由 robot_state_publisher 广播 (URDF)
 │              │
 │              └─ laser_link ← 激光雷达, TF 由 robot_state_publisher 广播 (URDF)
```

---

> **文档版本**: v1.0  
> **最后更新**: 2026-06-07  
> **维护**: 如有任何里程计/定位相关修改，请同步更新本文档
