# FishBot Cartographer 建图功能设计方案

> 设计日期：2026-05-31  
> 工作空间：`~/chapt9_update/fishbot_ws/`

---

## 一、现状分析

### 1.1 现有文件

| 文件 | 状态 | 说明 |
|------|------|------|
| `fishbot_bringup/launch/ydlidar_cartographer.launch.py` | ✅ 已存在（原始项目自带） | 整合了雷达+odom2tf+URDF+Cartographer+rviz |
| `fishbot_cartographer.lua` | ❌ **不存在** | Cartographer 核心配置文件，当前 launch 引用了但实际缺失 |
| `/home/ke/cartographer_config/` | ❌ **不存在** | launch 中默认的配置目录 |

### 1.2 已具备的条件

| 条件 | 详情 |
|------|------|
| **Cartographer ROS2** | ✅ 已安装 `ros-humble-cartographer-ros` |
| **激光雷达** | ✅ YDLIDAR (LifecycleNode + UDP桥接到 `/tmp/tty_laser`) |
| **编码器里程计** | ✅ `odom2tf` 节点，发布 `/odom` + TF `odom→base_footprint` |
| **URDF模型** | ✅ `fishbot.urdf` (base_footprint → base_link → laser_link) |
| **地图文件** | ✅ 已有多个地图 (`fishbot_map1`, `home`, `home2`, `home5`, `room3`) |
| **参考模板** | ✅ TurtleBot3 Cartographer 配置 (`turtlebot3_lds_2d.lua`) |

### 1.3 当前 launch 的问题

1. **缺少 lua 配置文件** — `fishbot_cartographer.lua` 不存在，无法运行
2. **未集成到主流程** — `bringup.launch.py` 和 `navigation2.launch.py` 都未引用此文件
3. **无保存地图功能** — launch 结束后没有 map_saver 保存地图
4. **无参数化** — 所有参数硬编码，不支持命令行覆盖
5. **tracking_frame 不明确** — launch 未指定 Cartographer 的 tracking_frame，需在 lua 中配置
6. **无建图/定位模式切换** — 建图和定位复用同一套参数，缺少区分

---

## 二、系统架构设计

### 2.1 数据流

```
┌──────────────────────────────────────────────────────────────────┐
│                       映射模式（Mapping Mode）                      │
│                                                                    │
│  ESP32 (micro-ROS)                                                 │
│  ├── 编码器 → /odom (x, y, yaw, vx, vy, wz)                       │
│  └── IMU → /imu_raw (含wz偏置) → imu_bias_remover → /imu_corrected │
│                                                                    │
│  YDLIDAR → /scan                                                   │
│       │                                                            │
│       ▼                                                            │
│  ╔═══════════════════════════════╗                                 │
│  ║    Cartographer SLAM Node     ║                                 │
│  ║                               ║                                 │
│  ║  输入: /scan + /odom          ║                                 │
│  ║  输出: map→odom TF            ║                                 │
│  ║        + submaps              ║                                 │
│  ╚═══════════════════════════════╝                                 │
│       │                                                            │
│       ▼                                                            │
│  cartographer_occupancy_grid_node                                   │
│  → 发布 /map (占用栅格地图)                                        │
│       │                                                            │
│       ▼                                                            │
│  map_saver → 保存为 .pgm + .yaml                                   │
│                                                                    │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│                    定位模式（Localization Mode）                   │
│                                                                    │
│  切换到 Navigation2 + AMCL（已有配置已完成优化）                    │
│  → 加载已保存的 .pgm/.yaml 地图                                   │
│  → AMCL + EKF 融合定位                                             │
│                                                                    │
└──────────────────────────────────────────────────────────────────┘
```

### 2.2 TF 树设计

```
Cartographer 模式
    map ──(Cartographer SLAM)──→ odom ──(odom2tf)──→ base_footprint
                                                          │
                                                     base_link
                                                          │
                                                     laser_link

Navigation2 模式（已有）
    map ──(AMCL)──→ odom ──(EKF)──→ base_footprint
                                        │
                                   base_link
                                        │
                                   laser_link
```

### 2.3 关键参数

| Cartographer 参数 | 推荐值 | 说明 |
|-------------------|--------|------|
| `tracking_frame` | `base_link` | 无独立 IMU link，以底盘为跟踪帧 |
| `published_frame` | `odom` | 发布 odom 帧 |
| `provide_odom_frame` | `false` | 由 odom2tf 提供 odom TF |
| `use_odometry` | `true` | 使用编码器里程计辅助扫描匹配 |
| `num_laser_scans` | `1` | 单激光 |
| `use_imu_data` | `false` | 无 IMU link，不使用 IMU 数据 |
| `min_range` | `0.12` | YDLIDAR 最小有效距离 |
| `max_range` | `8.0` | YDLIDAR 最大有效距离 |
| `use_online_correlative_scan_matching` | `true` | 实时相关性扫描匹配 |

---

## 三、实施步骤

### 步骤 1：创建 Cartographer Lua 配置文件

**目标文件**：`fishbot_bringup/config/fishbot_cartographer.lua`

基于 TurtleBot3 模板，适配 FishBot 硬件：
- 无独立 IMU link → `tracking_frame = "base_link"`
- 有编码器里程计 → `use_odometry = true`
- 只有 2D 激光 → 单激光配置
- 针对真实环境优化参数

### 步骤 2：重写建图 Launch 文件

**目标文件**：`fishbot_bringup/launch/cartographer_slam.launch.py`

新 launch 文件特性：
1. **参数化设计**：`use_sim_time`、`configuration_basename`、`resolution`、`publish_period_sec` 均可通过命令行覆盖
2. **集成 map_saver**：建图完成后可一键保存地图
3. **可选 rviz**：通过 `use_rviz` 参数控制是否启动可视化
4. **清晰的节点组织**：依赖关系明确

### 步骤 3：创建地图保存脚本

**目标文件**：`fishbot_bringup/fishbot_bringup/save_map.py`

CLI 工具：
```bash
ros2 run fishbot_bringup save_map --name my_map --dir ~/maps
```

本质是封装 `ros2 run nav2_map_server map_saver_cli -f <path>` 并提供路径管理。

### 步骤 4：集成到系统工作流

补充 `navigation2.launch.py`，支持通过参数切换建图/定位模式：
```bash
# 建图模式
ros2 launch fishbot_bringup cartographer_slam.launch.py

# 定位导航模式（已有）
ros2 launch fishbot_navigation2 navigation2.launch.py
```

---

## 四、文件清单

| # | 操作 | 文件路径 | 说明 |
|:-:|:----:|---------|------|
| 1 | **新建** | `fishbot_bringup/config/fishbot_cartographer.lua` | Cartographer SLAM配置 |
| 2 | **新建** | `fishbot_bringup/launch/cartographer_slam.launch.py` | 建图启动文件（替换原有方案） |
| 3 | **新建** | `fishbot_bringup/fishbot_bringup/save_map.py` | 地图保存脚本 |
| 4 | 保留 | `fishbot_bringup/launch/ydlidar_cartographer.launch.py` | 保留为备份参考，不删除 |
| 5 | 保留 | `fishbot_bringup/CMakeLists.txt` | 无需修改（Python模块已启用） |
| 6 | 保留 | `fishbot_bringup/launch/bringup.launch.py` | 基础启动不受影响 |
| 7 | 保留 | `fishbot_bringup/launch/urdf2tf.launch.py` | URDF启动不受影响 |

---

## 五、使用流程

### 5.1 建图模式

```bash
# 终端1：启动底盘基础功能
source ~/chapt9_update/fishbot_ws/install/setup.bash
ros2 launch fishbot_bringup bringup.launch.py

# 终端2：启动 Cartographer 建图
ros2 launch fishbot_bringup cartographer_slam.launch.py
# 或使用自定义配置：
ros2 launch fishbot_bringup cartographer_slam.launch.py \
    configuration_basename:=fishbot_cartographer.lua \
    resolution:=0.05

# 遥控机器人走遍房间（可使用 keyboard_teleop 或 Web UI）

# 终端3：保存地图
ros2 run fishbot_bringup save_map --name my_room
# 或手动：
ros2 run nav2_map_server map_saver_cli -f ~/chapt9_update/fishbot_ws/src/fishbot_navigation2/maps/my_room
```

### 5.2 定位导航模式（已有）

```bash
# 使用刚保存的地图
ros2 launch fishbot_navigation2 navigation2.launch.py \
    map:=~/chapt9_update/fishbot_ws/src/fishbot_navigation2/maps/my_room.yaml
```

### 5.3 系统运行拓扑

```
                 bringup.launch.py (常驻后台)
                ┌─────────────────────────┐
                │  urdf2tf                │
                │  odom2tf                │
                │  micro_ros_agent (8888) │
                │  ros_serial2wifi        │
                │  ydlidar (延时5s)       │
                └─────────────────────────┘
                          │
         ┌────────────────┴────────────────┐
         ▼                                  ▼
  cartographer_slam.launch.py      navigation2.launch.py
  ┌───────────────────────┐       ┌──────────────────────┐
  │ Cartographer Node     │       │ imu_bias_remover     │
  │ Occupancy Grid Node   │       │ EKF Node             │
  │ RViz2                 │       │ Nav2 Stack           │
  │ (map_saver手动保存)   │       │ AMCL                 │
  └───────────────────────┘       │ RViz2                │
                                  └──────────────────────┘
```

---

## 六、Cartographer lua 配置详解

### 6.1 核心参数说明

```lua
-- ==================== 坐标帧配置 ====================
options = {
  map_frame = "map",                    -- 全局地图坐标系
  tracking_frame = "base_link",         -- 跟踪帧（传感器所在帧）
  published_frame = "odom",             -- Cartographer发布的里程计帧
  odom_frame = "odom",                  -- 里程计坐标系
  provide_odom_frame = false,           -- 不提供odom帧（由odom2tf提供）
  publish_frame_projected_to_2d = true, -- 投影到2D平面
}

-- ==================== 传感器配置 ====================
-- 使用编码器里程计辅助扫描匹配
use_odometry = true

-- 单束激光扫描
num_laser_scans = 1

-- IMU数据：不使用（fishbot无独立imu_link，IMU只通过EKF提供wz）
-- 如果后续添加了imu_link到URDF中，可以改为true
use_imu_data = false

-- ==================== 扫描匹配参数 ====================
TRAJECTORY_BUILDER_2D.min_range = 0.12       -- YDLIDAR最小有效距离
TRAJECTORY_BUILDER_2D.max_range = 8.0        -- YDLIDAR最大有效距离
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0   -- 无数据时的默认射线长度

-- 启用实时相关性扫描匹配（提高精度，但增加CPU开销）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- ==================== 运动滤波 ====================
-- 只有移动超过阈值时才插入新的位姿
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- 约5.7°
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1          -- 10cm

-- ==================== 回环检测 ====================
-- 回环约束的最小匹配分数（提高可减少误匹配，但降低召回率）
POSE_GRAPH.constraint_builder.min_score = 0.65

-- 全局定位回环的最小匹配分数（高于局部回环，避免误匹配）
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- ==================== 优化参数 ====================
-- 优化频率：每10个节点优化一次位姿图
-- 设为0表示只优化一次（仅建图结束时）
-- POSE_GRAPH.optimize_every_n_nodes = 0
```

### 6.2 参数调优建议

| 场景 | `min_range` | `max_range` | `use_online_csm` | `min_score` |
|------|:-----------:|:-----------:|:-----------------:|:-----------:|
| 小房间 (<20m²) | 0.12 | 5.0 | true | 0.6 |
| 大房间 (20-100m²) | 0.12 | 8.0 | true | 0.65 |
| 走廊/长直道 | 0.12 | 10.0 | true | 0.7 |
| 快速建图（牺牲精度） | 0.12 | 5.0 | false | 0.55 |

---

## 七、与现有 Navigation2 的对比

| 特性 | Cartographer SLAM (本方案) | Navigation2 AMCL (已有) |
|------|:------------------------:|:----------------------:|
| **目的** | 创建新地图 | 在已有地图上定位导航 |
| **实时定位** | ✅ 同时定位和建图 | ✅ 仅定位 |
| **需要现有地图** | ❌ 不需要 | ✅ 必须加载 .pgm/.yaml |
| **输出** | .pgm + .yaml 地图文件 | /map → /odometry/filtered |
| **回环检测** | ✅ 有 | ❌ 无 |
| **CPU占用** | 较高（位姿图优化） | 较低（粒子滤波） |
| **适用场景** | 初次建图、环境变更后重绘 | 日常导航运行 |
| **里程计依赖** | 辅助（非必须） | 强依赖（粒子预测） |

---

## 八、建图操作最佳实践

### 8.1 建图前的准备

1. ✅ 确认激光雷达能正常旋转（`ros2 topic echo /scan` 有数据）
2. ✅ 确认里程计输出正常（`ros2 topic echo /odom` 数据合理）
3. ✅ 确认 TF 树完整（`ros2 run tf2_tools view_frames.py`）
4. ✅ 机器人充满电（建图过程可能需要 10-30 分钟）

### 8.2 建图过程建议

1. **缓慢移动** — 速度不超过 0.3 m/s，急停/急转会导致扫描匹配失败
2. **走"U"形路线** — 经过已探索区域形成回环，显著提高地图精度
3. **确保重叠** — 相邻路径保持 30-50% 重叠，Cartographer 需要持续的地图匹配
4. **回环是关键** — 回到已探索区域触发回环检测，修正累积误差
5. **先粗后细** — 第一次快速走全程建立全局框架，第二次慢速填补细节

### 8.3 常见问题排查

| 现象 | 可能原因 | 解决方案 |
|------|---------|---------|
| 地图严重扭曲 | 里程计漂移过大，或 `use_odometry=false` | 开启 `use_odometry=true`，检查编码器 |
| 建图过程中定位丢失 | 运动过快或激光被遮挡 | 减速，避免急转弯 |
| 回环后地图跳变 | `min_score` 过低导致误匹配 | 提高 `min_score` 到 0.7 |
| 地图有大量噪声点 | `max_range` 过大或 `min_range` 过小 | 限制 `max_range=5.0`, `min_range=0.15` |
| CPU 占用过高 | `use_online_csm=true` 或优化频率过高 | 关闭 online_csm，降低 `optimize_every_n_nodes` |
| 建图完成保存的地图空白 | Cartographer 输出 TF 但 map_server 没订阅对的话题 | 确保 `cartographer_occupancy_grid_node` 在运行 |
