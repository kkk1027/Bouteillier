# FishBot 深度相机 3D 建图与导航 — 可拓展性分析

> 日期：2026-06-07  
> 目标：评估现有 FishBot 架构对引入深度相机（RGB-D）做 3D SLAM + 3D 导航的可拓展性  
> 范围：仅 3D 建图与导航，不涉及 VLN（视觉语言导航）

> 📎 **相关文档**: [RPI5_MIGRATION_FEASIBILITY.md](./RPI5_MIGRATION_FEASIBILITY.md) — 树莓派5迁移 + 深度相机可行性分析（综合：硬件对比、ARM64兼容性、性能预估、迁移步骤）

---

## 总评

| 维度 | 评分 | 说明 |
|------|:----:|------|
| URDF / TF 树 | ⭐⭐⭐⭐⭐ | 极其清晰，加 camera link 只需 3 行 |
| Nav2 代价地图 | ⭐⭐⭐⭐☆ | 天然支持 VoxelLayer，历史已预留配置 |
| SLAM 替换 | ⭐⭐⭐⭐☆ | RTAB-Map 可直接输出 Nav2 兼容的占据栅格地图 |
| EKF 融合 | ⭐⭐⭐⭐☆ | 支持多传感器，加视觉里程计只需 1 行 |
| 性能裕量 | ⭐⭐⭐☆☆ | Intel i5，2 核 VM，RTAB-Map 需调参 |
| ESP32 端 | ⭐⭐⭐⭐⭐ | 深度相机通过 USB 直连 SBC，ESP32 无需改动 |

**结论：可拓展性高，改动集中在 ROS2 配置层，无需重构任何核心代码。**

---

## 一、现有架构逐层分析

### 1.1 URDF / TF 树 —— 改动极小

当前 TF 树：
```
base_footprint → base_link → laser_link
```

**需要添加**：
```xml
<!-- depth camera link —— 加在 fishbot.urdf 中，base_link 上方 -->
<link name="depth_camera_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.09 0.025 0.025" />
    </geometry>
    <material name="dark_gray">
      <color rgba="0.3 0.3 0.3 1.0" />
    </material>
  </visual>
</link>

<joint name="depth_camera_joint" type="fixed">
  <parent link="base_link" />
  <child link="depth_camera_link" />
  <origin xyz="0.12 0 0.10" rpy="0 0 0" />
</joint>
```

**影响范围**：仅 `fishbot_description/urdf/fishbot.urdf` 一个文件，3 行 XML。  
**风险评估**：零风险，固定 joint 不引入新的运动学约束，不影响现有 TF 广播。

修正后的 TF 树：
```
base_footprint → base_link → laser_link
                          → depth_camera_link
```

### 1.2 SLAM —— Cartographer 2D → RTAB-Map RGB-D

**现状**：Cartographer 纯 2D（`use_trajectory_builder_2d = true`，`num_point_clouds = 0`）。Cartographer 虽有 3D 模式但主要面向 LiDAR 3D 点云，对 RGB-D 支持不成熟。

**推荐方案**：RTAB-Map（`ros-humble-rtabmap-*`，apt 已有包，未安装）

RTAB-Map 的优势：
- RGB-D SLAM 的工业标准，ROS2 生态成熟
- 输出标准 `/map` 话题（`nav_msgs/OccupancyGrid`），Nav2 无需任何修改即可消费
- 同时输出稠密 3D 点云地图（`/rtabmap/cloud_map`）用于可视化
- 支持视觉回环检测（比 Cartographer 激光回环更鲁棒）
- 可集成视觉里程计（VO）作为额外里程计源

**架构对照**：

```
现状：
  /scan (LaserScan) → Cartographer 2D → /map (OccupancyGrid) + map→odom TF

改后：
  /scan (LaserScan) ─────────────────┐
  /depth/image_raw (深度图)          ├→ RTAB-Map → /map (OccupancyGrid)
  /rgb/image_raw (RGB图)             │              + map→odom TF
  /depth/camera_info (内参)          │              + /rtabmap/cloud_map (3D点云)
  /odom (里程计) ────────────────────┘
```

**关键点**：
- RTAB-Map 可以**同时融合激光和深度相机**，不是替代而是增强
- `/map` 输出格式不变，Nav2 无需改动
- Cartographer launch 文件直接替换为 RTAB-Map launch，其他模块不受影响

### 1.3 Nav2 代价地图 —— VoxelLayer 支持已预留

**现状**：local_costmap 使用 `ObstacleLayer`
```yaml
plugins: ["obstacle_layer", "inflation_layer"]
```

**关键发现**：`nav2_params.yaml.bak` 中**原本就有 VoxelLayer 配置**：
```yaml
plugins: ["voxel_layer", "inflation_layer"]
voxel_layer:
  plugin: "nav2_costmap_2d::VoxelLayer"
  enabled: True
  publish_voxel_map: True
  origin_z: 0.0
  z_resolution: 0.05
  z_voxels: 16
  max_obstacle_height: 2.0
  observation_sources: scan
  scan:
    topic: /scan
    # ... 激光参数 ...
```

这说明项目之前就考虑过 3D 感知。切换到 VoxelLayer 后，再加入深度相机点云作为第二个观测源：

```yaml
plugins: ["voxel_layer", "inflation_layer"]
voxel_layer:
  plugin: "nav2_costmap_2d::VoxelLayer"
  enabled: True
  publish_voxel_map: True
  origin_z: 0.0
  z_resolution: 0.05
  z_voxels: 16
  max_obstacle_height: 2.0
  mark_threshold: 0
  observation_sources: scan depth_camera
  scan:
    topic: /scan
    data_type: "LaserScan"
    marking: True
    clearing: True
    max_obstacle_height: 2.0
    raytrace_max_range: 3.0
    raytrace_min_range: 0.30
    obstacle_max_range: 2.5
    obstacle_min_range: 0.35
  depth_camera:              # ← 新增
    topic: /rtabmap/obstacles
    data_type: "PointCloud2"
    marking: True
    clearing: True
    max_obstacle_height: 2.0
    min_obstacle_height: 0.05
    obstacle_max_range: 4.0
    obstacle_min_range: 0.2
```

**改动量**：`nav2_params.yaml` 一个文件，约 20 行。

### 1.4 EKF —— 可融合视觉里程计

**现状**：
```yaml
two_d_mode: true          # 2D 模式
odom0: /odom_ekf           # 编码器里程计
imu0: /imu_corrected       # IMU wz
```

**需要调整**：
```yaml
two_d_mode: false          # 改为 3D 模式（或保持 2D 也可以，取决于需求）

# 可选：添加视觉里程计作为第三个输入源
odom1: /rtabmap/odom       # RTAB-Map 视觉里程计
odom1_config: [true, true, true, true, true, true,  # 6DOF
               false, false, false, false, false, false, false, false, false]
odom1_differential: false
odom1_relative: false
odom1_queue_size: 10
```

**注意**：如果只做 3D 建图和障碍物检测（不要求机器人 6DOF 运动），`two_d_mode: true` 完全可以保留。3D 信息主要用于**地图和感知**，机器人控制仍然是 2D（差速底盘）。

### 1.5 ESP32 端 —— 无需改动

深度相机通过 **USB 直连 SBC（Intel i5）**，ESP32 端完全不参与。现有的：
- micro-ROS UDP 通信
- 编码器里程计
- IMU 数据
- PID 电机控制

全部保持不变。ESP32-CAM（用于跟随模式）是独立硬件，不影响深度相机集成。

---

## 二、完整集成链路

```
┌──────────────────────────────────────────────────────────────────────┐
│                        硬件层                                         │
│                                                                       │
│  Intel RealSense D435i (或 OAK-D Lite)                               │
│  ├─ RGB 图像: 640×480 @30Hz                                          │
│  ├─ 深度图:   640×480 @30Hz                                          │
│  └─ IMU:     内置 6 轴 (可选替代 MPU6050)                             │
│                                                                       │
│  USB 3.0 ──→ SBC (Intel i5-11400H, 15GB RAM)                        │
└──────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────────┐
│                       ROS2 驱动层 (新增)                              │
│                                                                       │
│  realsense2_camera_node (或 depthai_examples)                        │
│  ├─ /camera/color/image_raw   (sensor_msgs/Image)                    │
│  ├─ /camera/depth/image_rect_raw (sensor_msgs/Image)                 │
│  ├─ /camera/color/camera_info (sensor_msgs/CameraInfo)               │
│  └─ /camera/depth/camera_info (sensor_msgs/CameraInfo)               │
└──────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────────┐
│                      SLAM 层 (Cartographer → RTAB-Map)               │
│                                                                       │
│  rtabmap node                                                         │
│  输入:                                                                │
│    /scan, /odom, /camera/rgb/..., /camera/depth/..., /camera/info    │
│  输出:                                                                │
│    /map (OccupancyGrid)        → Nav2 直接消费                        │
│    /tf (map→odom)              → TF 树                                │
│    /rtabmap/cloud_map          → 3D 可视化                            │
│    /rtabmap/odom               → 可选的视觉里程计                      │
└──────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────────┐
│             定位层 (AMCL — 不变)                                      │
│                                                                       │
│  输入: /scan + /map + /tf                                             │
│  输出: /tf (map→odom 修正)                                           │
│  注意: AMCL 仍用 2D 激光定位，不依赖深度相机                          │
└──────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────────┐
│           代价地图层 (ObstacleLayer → VoxelLayer)                     │
│                                                                       │
│  local_costmap 观测源:                                                │
│    /scan (LaserScan)       → 2D 障碍物                               │
│    /rtabmap/obstacles (PointCloud2) → 3D 障碍物 (悬空物)             │
│                                                                       │
│  3D 能力: 检测桌面、低矮障碍物等 2D LiDAR 无法看到的物体              │
└──────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────────┐
│              规划控制层 (Nav2 — 不变)                                 │
│                                                                       │
│  SmacPlanner2D + RegulatedPurePursuit                                │
│  → 仍然是 2D 规划，但代价地图包含 3D 信息                            │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 三、改动清单

### 3.1 必须改动的文件

| # | 文件 | 改动 | 风险 |
|---|------|------|:----:|
| 1 | `fishbot_description/urdf/fishbot.urdf` | 添加 `depth_camera_link` + joint (3行) | 无 |
| 2 | `fishbot_bringup/config/fishbot_cartographer.lua` | 不再需要（RTAB-Map 替代） | — |
| 3 | `fishbot_navigation2/config/nav2_params.yaml` | local_costmap 切到 VoxelLayer + 加 PointCloud2 观测源 (~20行) | 低 |
| 4 | 新建 `fishbot_bringup/launch/rtabmap_slam.launch.py` | RTAB-Map SLAM 启动文件 (~100行) | 低 |
| 5 | 新建 `fishbot_bringup/config/rtabmap_params.yaml` | RTAB-Map 参数配置 (~80行) | 低 |
| 6 | 新建 `fishbot_bringup/launch/depth_camera.launch.py` | RealSense/OAK-D 驱动启动 (~40行) | 低 |

### 3.2 可选改动

| # | 文件 | 改动 | 收益 |
|---|------|------|------|
| 7 | `fishbot_navigation2/config/ekf.yaml` | `two_d_mode: false` + 添加 odom1 (视觉里程计) | 3D 定位 |
| 8 | `fishbot_bringup/launch/bringup.launch.py` | 加入深度相机驱动的 TimerAction | 一键启动 |
| 9 | `fishbot_navigation2/config/nav2_params.yaml` | global_costmap 也加 VoxelLayer | 全局 3D 感知 |

### 3.3 不需要改动的

- ✅ ESP32 固件（`fishbot_motion_control/`）
- ✅ micro-ROS agent
- ✅ odom2tf / odom_covariance_injector
- ✅ AMCL 定位
- ✅ SmacPlanner2D / RegulatedPurePursuit
- ✅ BehaviorTree 导航逻辑
- ✅ AI patrol / follow 模式
- ✅ `autopatrol_robot` 全部代码

---

## 四、推荐硬件

| 型号 | 价格 | 分辨率 | FOV | 推荐度 |
|------|------|--------|-----|:------:|
| **Intel RealSense D435i** | ~$200 | 1280×720 @30Hz | 87°×58° | ⭐⭐⭐⭐⭐ |
| OAK-D Lite | ~$150 | 640×480 @30Hz | 86°×57° | ⭐⭐⭐⭐ |
| Orbbec Astra Pro | ~$150 | 640×480 @30Hz | 60°×49° | ⭐⭐⭐ |

推荐 **D435i**：ROS2 驱动最成熟（`ros-humble-realsense2-camera`），内置 IMU 可替代 MPU6050，全局快门，室外/室内都可用。

---

## 五、性能评估

### 5.1 当前资源占用（粗略）

| 组件 | CPU 占用估计 |
|------|:-----------:|
| Cartographer 2D | 15-25% (1核) |
| Nav2 (AMCL + Planner + Controller) | 10-15% |
| EKF + TF | 2-5% |
| micro-ROS + ydlidar 桥接 | 5-10% |
| **合计** | ~35-50% (1核) |

### 5.2 增加深度相机后

| 新增组件 | CPU 占用估计 |
|----------|:-----------:|
| realsense2_camera_node | 5-10% |
| RTAB-Map (替换 Cartographer) | 20-35% (1核) |
| VoxelLayer (替代 ObstacleLayer) | ~5% 额外 |
| **增量** | ~15-30% |

**SBC 评估**：Intel i5-11400H 有 6 核 12 线程，VM 分配 2 核。如果 RTAB-Map 参数合理（降低点云密度、降低回环检测频率），2 核可承受。如有压力，可扩大 VM 的 vCPU 分配。

### 5.3 RTAB-Map 性能调优要点

```yaml
# 关键参数 —— 在性能和效果间平衡
Rtabmap/ImageRate: 2.0           # 每秒处理 2 帧（不用 30fps）
Rtabmap/DetectionRate: 1.0       # 回环检测每秒 1 次
Grid/FromDepth: true             # 从深度图直接生成占据栅格（快）
Mem/UseOdomFeatures: true        # 用里程计辅助回环检测
RGBD/OptimizeFromGraphEnd: false # 减少位姿图优化频率
```

---

## 六、实施路径

### Phase 1：基础集成（1-2 天）

1. 连接深度相机，确认 USB 识别和驱动
2. 安装 `ros-humble-realsense2-camera`（或对应驱动）
3. 编写 `depth_camera.launch.py`，发布 RGB/深度/CameraInfo
4. 在 URDF 添加 `depth_camera_link`
5. 在 RViz 中验证 TF 树和图像话题

### Phase 2：SLAM 替换（1-2 天）

1. 安装 `ros-humble-rtabmap-ros` 全家桶
2. 编写 `rtabmap_slam.launch.py` 和 `rtabmap_params.yaml`
3. 同时订阅 `/scan`（激光）+ `/camera/depth/...`（深度）
4. 验证 `/map` 话题输出格式与 Cartographer 兼容
5. 测试建图效果（在小范围环境）

### Phase 3：Nav2 3D 感知（1 天）

1. `nav2_params.yaml` 切换 VoxelLayer
2. 添加 PointCloud2 观测源
3. 测试障碍物检测（特别是悬空物、低于激光平面的物体）

### Phase 4：调优与验证（1-2 天）

1. 整机联合测试（建图 + 导航）
2. 性能调优（RTAB-Map 参数、VoxelLayer 分辨率）
3. 3D 地图可视化验证
4. 必要时调整 EKF 为 3D 模式

---

## 七、风险与缓解

| 风险 | 概率 | 影响 | 缓解措施 |
|------|:----:|:----:|----------|
| RTAB-Map CPU 过高 | 中 | 卡顿 | 降低 ImageRate 到 1-2Hz，减少 z_voxels |
| USB 带宽不足 | 低 | 丢帧 | 降低相机分辨率到 640×480 |
| TF 树冲突 | 低 | 定位跳变 | RTAB-Map 发布的 map→odom 与 AMCL 冲突，只需用其中一个 |
| 深度相机盲区 | 中 | 近处障碍漏检 | 保留 `/scan` 激光作为主传感器，深度相机作辅助 |
| VoxelLayer 内存增长 | 低 | OOM | `z_voxels: 16` 保持原配置，不盲目增大 |

---

## 八、总结

> FishBot 项目的架构对深度相机 3D 建图与导航的**可拓展性很高**。

**核心原因**：

1. **模块化设计**：SLAM、定位、代价地图、规划各自独立，替换任一模块不影响其他
2. **Nav2 原生支持 3D**：VoxelLayer 在 Nav2 中是一等公民，配置即用
3. **RTAB-Map 兼容 Nav2**：输出标准 OccupancyGrid，无适配成本
4. **URDF 极简**：加一个固定 joint 无需任何运动学分析
5. **ESP32 不受影响**：深度相机直连 SBC，硬件架构天然解耦
6. **历史配置回归**：VoxelLayer 配置曾经存在于 .bak 中，说明项目早期就考虑过这条路

唯一需要注意的是 **CPU 裕量**（当前 VM 2 核），但通过 RTAB-Map 参数调优可以控制在可接受范围。
