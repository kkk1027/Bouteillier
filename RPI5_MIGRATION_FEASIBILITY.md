# FishBot 迁移至树莓派5 + 深度相机 — 综合可行性分析

> **分析日期**: 2026-05-31  
> **迁移目标**: PC 虚拟机 (Intel i5-11400H, 2 vCPU, 15GB RAM) → Raspberry Pi 5 (4×Cortex-A76, 4/8GB RAM)  
> **新增硬件**: 深度相机 (RealSense D435i / OAK-D Lite)  
> **运行模式**: 预建图 + 基于已知地图的导航（**不做实时 SLAM**）  
> **结论**: ✅ **项目可行，中等风险**。核心路径全部畅通，需注意CPU负载管理和内存优化。

---

## 1. 硬件对比：PC VM vs Raspberry Pi 5

| 维度 | 当前 PC VM | 树莓派 5 | 评估 |
|:---|:---|:---|:---|
| **CPU 架构** | x86_64 (Intel i5-11400H) | ARM64 (Cortex-A76) | 需全量 ARM64 重建 |
| **逻辑核心** | 2 vCPU (6C/12T 的子集) | 4 物理核 | 多核 ≈ 相当，单核 RPi5 ≈ 60% Intel |
| **单核性能 (Geekbench5)** | ~1400-1500 | ~800-900 | RPi5 单核弱约 40% |
| **多核性能 (Geekbench5)** | ~2800 (2 vCPU) | ~2700-3000 | **大致相当** ← 关键优势 |
| **内存** | 15 GB (DDR4) | 4 GB / 8 GB (LPDDR4X) | **选 8GB 版本**；4GB 紧张 |
| **存储** | VM 虚拟磁盘 | microSD / NVMe SSD | 大容量 A2 SD 卡可起步，长期建议 NVMe（详见 §1.2） |
| **GPU** | 无 (VM 直通有限) | VideoCore VII (无 CUDA) | 双方都无 CUDA，YOLO 都跑 CPU |
| **功耗** | 虚拟机宿主机共享 | ~8-15W (整机) | ✅ 显著降低 |
| **体积** | 台式机/笔记本 | 信用卡大小 | ✅ 可直接嵌入机器人 |

### 1.1 关键结论

RPi5 的多核性能（4 个物理核 @2.4GHz）与当前 2 vCPU 的 VM 大致持平。当前所有 ROS2 节点（AMCL, Nav2, micro-ROS, lidar driver 等）已在 **2 核** 环境下稳定运行 — RPi5 的 **4 物理核** 反而可能表现更好。

---

## 2. 逐组件迁移分析

### 2.1 组件清单与 ARM64 兼容性矩阵

| 组件 | 当前运行位置 | ARM64 可用性 | 迁移难度 | 备注 |
|:---|:---|:---|:---|:---|
| **ROS2 Humble Base** | PC VM | ✅ `arm64` 官方支持 | 🟢 极低 | Ubuntu 22.04 arm64 或 Raspberry Pi OS + ROS apt |
| **Cartographer** | PC VM | ✅ `arm64` 包可用 | 🟢 低 | 仅建图时用，导航时不需要 |
| **Navigation2 (AMCL+Planner+Controller)** | PC VM | ✅ `arm64` 包可用 | 🟢 低 | 核心导航栈，ARM64 原生二进制 |
| **robot_localization (EKF)** | PC VM | ✅ `arm64` 包可用 | 🟢 低 | C++ 组件，ARM64 原生 |
| **ydlidar_ros2_driver** | PC VM | ✅ 源码编译 | 🟢 低 | 纯 C++，colcon build 直接编译 |
| **micro-ROS Agent** | PC VM | ⚠️ 需 ARM64 重编译 | 🟡 中 | 源码在 `fishbot_ws/src/micro-ROS-Agent/` |
| **ros_serial2wifi** | PC VM | ⚠️ 需 ARM64 重编译 | 🟡 中 | 自研包，C++ 源码 |
| **autopatrol_robot** | PC VM | ✅ Python (架构无关) | 🟢 低 | 纯 Python 包 |
| **DeepSeek AI API** | 云端 | ✅ 云端 API | 🟢 无 | `api.deepseek.com/v1`，零本地计算 |
| **YOLOv8n (跟随模式)** | PC VM | ⚠️ ARM64 PyTorch | 🟡 中 | CPU 推理，RPi5 上约 3-5 FPS |
| **ESP32 固件编译 (PlatformIO)** | PC VM | ✅ ARM64 apt 可用 | 🟢 低 | 交叉编译到 ESP32 |
| **深度相机驱动** | — (新增) | ✅ `arm64` 包可用 | 🟢 低 | RealSense / OAK-D 均有 ARM64 支持 |

### 2.2 详细分析

#### 🔴 高优先级：micro-ROS Agent（ARM64 重编译）

```
源码位置: fishbot_ws/src/micro-ROS-Agent/
构建方式: CMake (ExternalProject)
当前二进制: x86_64 ELF, 动态链接到 ROS2 Humble libraries
```

**解决路径**：
1. 在 RPi5 上 `colcon build --packages-select micro_ros_agent`
2. ExternalProject 会自动下载 micro-ROS 源码并编译
3. 验证通过 ~5 分钟

**风险**：micro_ros_agent 是一个 ExternalProject（非标准 ROS2 包结构），在 ARM64 上可能需要调整 CMake 参数。

#### 🔴 高优先级：ros_serial2wifi（自研包重编译）

```
源码位置: fishbot_ws/src/ros_serial2wifi/
类型: C++ ROS2 包
构建方式: colcon build
```

**解决路径**：标准 colcon 编译，与 x86 完全相同。

#### 🟡 中优先级：PyTorch / YOLOv8n（ARM64 CPU 推理）

```
当前: torch 2.12.0+cpu (x86_64 wheel)
ARM64: torch 2.12.0 有 aarch64 wheel (pip install torch)
使用位置: /home/ke/detection_ws/src/esp32_cam_detector/
模型: yolov8n.pt (~6 MB)
推理分辨率: 320×256 (已优化)
```

**性能预估**：
- x86 VM (2 vCPU): 约 8-12 FPS (YOLOv8n on CPU)
- RPi5 (4×A76): 约 3-5 FPS (YOLOv8n on CPU)
- 跟随模式对帧率要求：2-5 FPS 即可（自适应检测频率已实现）

**结论**：可行但帧率降低。如果性能不足，可考虑：
- 进一步降低分辨率至 224×224
- 使用 YOLOv8n 导出为 ONNX + ONNX Runtime（ARM64 优化更好）
- 或使用 Coral USB TPU 加速（~$60，30 FPS）

#### 🟢 低/无影响：DeepSeek AI Agent

```
实现: LangGraph + ChatOpenAI(base_url="https://api.deepseek.com/v1")
用途: 自然语言指令解析、工具调用、巡逻任务编排
本地依赖: 仅 Python (langgraph, langchain, openai)
```

**零架构依赖** — DeepSeek 是完全的云端 API，RPi5 上只需要 HTTP 请求能力。无任何迁移成本。

#### 🟢 低影响：ESP32 固件开发

PlatformIO 在 ARM64 Ubuntu 上可通过 apt 安装，支持 ESP32 交叉编译工具链。构建环境与 x86 完全一致。

---

## 3. 深度相机集成方案

### 3.1 相机选型

| 特性 | Intel RealSense D435i | OAK-D Lite | 备注 |
|:---|:---|:---|:---|
| **深度技术** | 主动立体红外 | 立体视觉 + 片上 AI | |
| **ROS2 驱动** | `ros-humble-realsense2-camera` (ARM64 ✅) | `ros-humble-depthai-ros` (ARM64 ✅) | |
| **IMU 内置** | ✅ BMI055 | ❌ (需 OAK-D Pro) | D435i 可替代当前 MPU6050 |
| **功耗** | ~3.5W | ~2.5W | RPi5 USB 3.0 可供电 |
| **价格** | ~$250 | ~$150 | |
| **ARM64 成熟度** | ⭐⭐⭐⭐⭐ 极成熟 | ⭐⭐⭐⭐ 成熟 | D435i 社区更大 |
| **推荐** | ✅ **首选** | 备选 (预算敏感) | |

### 3.2 推荐：Intel RealSense D435i

**理由**：
1. ARM64 (RPi5) 上驱动极度成熟，Ubuntu 官方仓库提供预编译包
2. 内置 IMU，可替代当前 MPU6050，减少硬件复杂度
3. 主动红外投影，在低纹理/暗光环境仍可工作
4. ROS2 集成文档完善，`realsense2_camera_node` 即插即用

### 3.3 URDF 改动（最小化）

当前 URDF（`fishbot.urdf`）只有 `base_footprint → base_link → laser_link`，添加深度相机仅需 3 行 XML：

```xml
<!-- 在 fishbot.urdf 的 base_link 下添加 -->
<joint name="depth_camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="depth_camera_link"/>
  <origin xyz="0.15 0 0.25" rpy="0 0 0"/>
</joint>
<link name="depth_camera_link"/>
```

### 3.4 导航栈改动（最小化）

当前 `local_costmap` 使用 `ObstacleLayer`（仅 2D 激光）。深度相机引入后，切换到 `VoxelLayer`（支持 3D 点云）：

```yaml
# local_costmap plugins 改为:
plugins: ["voxel_layer", "inflation_layer"]

voxel_layer:
  plugin: "nav2_costmap_2d::VoxelLayer"
  enabled: True
  z_resolution: 0.05
  z_voxels: 16
  max_obstacle_height: 2.0
  observation_sources: scan pointcloud    # 同时使用激光+深度
  scan:
    topic: /scan
    data_type: "LaserScan"
    marking: True
    clearing: True
  pointcloud:
    topic: /depth_camera/points            # RealSense 点云
    data_type: "PointCloud2"
    marking: True
    clearing: True
    min_obstacle_height: 0.05              # 地面过滤
    max_obstacle_height: 2.0
```

**已有基础**：`nav2_params.yaml.bak` 中已有完整的 VoxelLayer 配置模板，仅需添加 `pointcloud` 观测源。

### 3.5 建图工作流（离线 → 在线分离）

```
┌─────────────────────────────────────────────────────────┐
│  阶段 1：建图（在 PC 上完成，不占 RPi5 资源）            │
│                                                         │
│  RTAB-Map (3D) 或 Cartographer (2D)                     │
│  ├── 运行 SLAM 采集环境地图                              │
│  ├── 保存为 .pgm + .yaml (2D occupancy grid)            │
│  └── 或保存为 .db (RTAB-Map 3D 数据库)                  │
│                                                         │
│  阶段 2：导航（在 RPi5 上运行）                          │
│                                                         │
│  Navigation2 + AMCL                                     │
│  ├── map_server ← 预建 .pgm/.yaml 地图                  │
│  ├── AMCL ← /scan (激光定位)                            │
│  ├── local_costmap (VoxelLayer) ← /scan + 深度点云      │
│  └── Planner + Controller ← 路径规划+执行               │
│                                                         │
│  ❌ 不运行 Cartographer / RTAB-Map SLAM                  │
│  ❌ 不做实时地图更新（静态环境假设）                      │
└─────────────────────────────────────────────────────────┘
```

---

## 4. RPi5 运行时性能预估

### 4.1 CPU 负载预估（多核分布）

| 节点 | 预估 CPU 占用 (RPi5) | 核心亲和性建议 |
|:---|:---|:---|
| **micro_ros_agent** (UDP) | 5-10% | 核心 0 |
| **ydlidar_ros2_driver** | 8-12% | 核心 0 |
| **realsense2_camera_node** | 15-25% | 核心 1 |
| **AMCL** (500 粒子) | 8-15% | 核心 2 |
| **Nav2 Planner + Controller** | 10-20% | 核心 2 |
| **local_costmap (VoxelLayer)** | 10-18% | 核心 3 |
| **global_costmap** | 3-5% | 核心 2 |
| **EKF (robot_localization)** | 2-5% | 核心 0 |
| **ros_serial2wifi** (TCP) | 3-5% | 核心 0 |
| **autopatrol_robot AI** | 1-3% (空闲) | 任意 |
| **YOLOv8n (跟随模式，可选)** | 30-50% | 核心 3 (按需) |
| **系统开销 (DDS/UDP 通信)** | 5-10% | — |
| **总计 (无跟随)** | **~60-85%** | ✅ 有余量 |
| **总计 (含跟随)** | **~90-135%** | ⚠️ 接近/超载 |

### 4.2 内存预估

| 组件 | 预估内存 |
|:---|:---|
| Ubuntu Server 22.04 (headless) | ~300 MB |
| ROS2 Humble Base + DDS | ~200 MB |
| AMCL + Nav2 | ~400 MB |
| Costmaps (3m×3m + global) | ~100 MB |
| Realsense2 驱动 + 点云缓冲 | ~300 MB |
| Python (autopatrol_robot + LangGraph) | ~200 MB |
| PyTorch + YOLOv8n (跟随模式) | ~500 MB |
| **总计 (无跟随)** | **~1.5 GB** |
| **总计 (含跟随)** | **~2.0 GB** |

**结论**：4GB 版本可运行无跟随模式（有 ~2.5GB 余量），8GB 版本可运行全部功能。**强烈建议 8GB 版本**。

### 4.3 关键优化建议

1. **AMCL 粒子数已优化**：当前 500→200 (min)，对 RPi5 影响很小
2. **local_costmap 尺寸已缩小**：3m×3m (原来 5m×5m)，计算量减半
3. **controller_frequency 已调整**：10 Hz（非 20 Hz）
4. **深度相机分辨率**：建议 640×480@15Hz（非 1280×720），节省 USB 带宽和 CPU
5. **跟随模式可设为可选**：通过 ROS2 参数动态启停 YOLO 节点

---

## 5. 操作系统选择

| 选项 | 优点 | 缺点 | 推荐度 |
|:---|:---|:---|:---|
| **Ubuntu 22.04 Server arm64** | ROS2 Humble 官方支持，apt 直接安装 | 需刷 Ubuntu 镜像 | ⭐⭐⭐⭐⭐ |
| **Raspberry Pi OS (Bookworm)** | 树莓派官方优化 | 需从 ROS apt 仓库手动添加 | ⭐⭐⭐ |
| **Ubuntu 24.04 + ROS2 Jazzy** | 更新版本 | 部分包不在 Humble 兼容列表中 | ⭐⭐ |

**推荐**：Ubuntu 22.04 Server arm64（[官方 RPi 镜像](https://ubuntu.com/download/raspberry-pi)），ROS2 Humble 第一类支持平台。

---

## 6. 迁移步骤（分阶段执行）

### 阶段 1：基础环境搭建（预计 2-3 小时）

```
1. 刷写 Ubuntu 22.04 Server arm64 到 microSD 卡（或 NVMe SSD）
2. 配置网络、SSH、swap (建议 4GB)
3. 如果是 SD 卡：应用 §1.2 的优化策略（日志→tmpfs、noatime、swappiness）
4. 安装 ROS2 Humble:
   $ sudo apt install ros-humble-ros-base ros-humble-navigation2 \
     ros-humble-slam-toolbox ros-humble-cartographer-ros \
     ros-humble-robot-localization ros-humble-xacro
4. 创建工作空间:
   $ mkdir -p ~/chapt9_update/fishbot_ws/src
5. 复制源码:
   $ rsync -avz ke@pc-vm:/mnt/data/chapt9_update/fishbot_ws/src/ ~/chapt9_update/fishbot_ws/src/
```

### 阶段 2：ARM64 编译（预计 1-2 小时）

```
6. 编译全部 ROS2 包:
   $ cd ~/chapt9_update/fishbot_ws
   $ colcon build --symlink-install --continue-on-error
7. 重点关注 micro_ros_agent 编译（ExternalProject）
8. 验证所有二进制产物:
   $ file install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent
   # 应输出: ELF 64-bit LSB executable, ARM aarch64
```

### 阶段 3：硬件对接（预计 2-3 小时）

```
9. 连接 YDLIDAR X4 (USB)
10. 连接 ESP32 (WiFi UDP 8888)
11. 连接 RealSense D435i (USB 3.0)
12. 连接 MPU6050 (I2C 或 D435i 内置 IMU)
13. 测试各传感器:
    $ ros2 topic list
    # 预期: /scan, /odom, /imu, /depth_camera/points 等
```

### 阶段 4：深度相机集成（预计 2-3 小时）

```
14. 安装 RealSense 驱动:
    $ sudo apt install ros-humble-realsense2-camera
15. 添加 depth_camera_link 到 fishbot.urdf
16. 切换到 VoxelLayer (从 .bak 恢复配置，添加 PointCloud2 观测源)
17. 校准 TF 变换 (base_link → depth_camera_link)
18. 测试 3D 避障:
    $ ros2 launch fishbot_navigation2 navigation.launch.py
    # 在 Rviz2 中观察 VoxelGrid 标记
```

### 阶段 5：预建地图导航（预计 1-2 小时）

```
19. 在 PC 上建图（Cartographer 或 RTAB-Map）
20. 保存地图:
    $ ros2 run nav2_map_server map_saver_cli -f ~/map
    # 生成 map.pgm + map.yaml
21. 将地图传输到 RPi5
22. 启动纯导航（无 SLAM）:
    $ ros2 launch fishbot_navigation2 navigation.launch.py map:=~/map.yaml
23. 验证 AMCL 定位 + 路径规划 + 深度相机避障
```

### 阶段 6：AI Agent + 跟随模式（预计 1-2 小时）

```
24. 安装 Python 依赖:
    $ pip install langgraph langchain langchain-openai flask-socketio
25. 安装 ARM64 PyTorch:
    $ pip install torch --index-url https://download.pytorch.org/whl/cpu
26. 配置 DeepSeek API Key（无需变更）
27. 测试 AI 指令导航
28. 测试跟随模式（可选，评估帧率是否可接受）
```

---

## 7. 风险矩阵

| 风险 | 概率 | 影响 | 缓解措施 |
|:---|:---|:---|:---|
| **micro_ros_agent ARM64 编译失败** | 中 | 高 — 核心通信中断 | 备选：使用 micro-ROS 官方 ARM64 Docker 镜像；或交叉编译 |
| **PyTorch ARM64 性能不足** | 中 | 中 — 跟随帧率低 | ONNX Runtime ARM64；降低分辨率；Coral TPU |
| **RPi5 USB 带宽瓶颈** | 低-中 | 中 — 深度相机+激光争用 | 使用 RPi5 USB 3.0 口；降低深度相机帧率 |
| **SD 卡 IO 瓶颈** | 中 | 中 — 日志/地图加载慢 | 使用 A2 等级 SD 卡；日志写入 tmpfs (#4.3 优化策略)；后续升级 NVMe SSD |
| **D435i USB 供电不足** | 低 | 低 — 相机断连 | 使用有源 USB Hub；RPi5 官方电源 (5V/5A) |
| **内存不足 (4GB 版本)** | 中 | 高 — 系统 OOM | 选 8GB 版本；增大 swap；关闭不必要的服务 |
| **ROS2 Humble arm64 包缺失** | 低 | 中 — 需源码编译 | 源码编译备选（已验证 colcon 工具链可用） |

---

## 8. 推荐硬件配置（高性价比方案）

### 8.1 🎯 核心推荐配置（甜点级）

> **一句话**：RPi5 8GB + A2 SD 卡起步 + D435i 深度相机，总新增成本约 **¥1,600-1,800**（复用现有激光雷达和底盘）。

| 类别 | 具体选型 | 参考价 (¥) | 必要性 | 说明 |
|:---|:---|:---|:---|:---|
| **主板** | Raspberry Pi 5, **8GB** | ~¥480 | 🔴 必须 | 4GB 版在深度相机 + 跟随模式下内存紧张，8GB 一步到位 |
| **存储** | Samsung PRO Plus 128GB A2 microSD | ~¥80 | 🔴 必须 | A2 等级保证随机读写性能；先用 SD 卡跑通，后续可加 NVMe（不浪费） |
| **电源** | 官方 27W USB-C (5V/5A) | ~¥70 | 🔴 必须 | D435i 通过 USB 供电，必须足额 5A 才能稳定驱动 |
| **深度相机** | Intel RealSense D435i | ~¥1,000-1,200 | 🟡 核心新增 | 内置 IMU 可替代 MPU6050；ARM64 驱动最成熟的深度相机 |
| **散热** | 官方主动散热器 或 带风扇金属外壳 | ~¥30-50 | 🟢 推荐 | RPi5 满载时被动散热不够，主动散热避免降频 |
| **USB Hub** | 有源 USB 3.0 Hub（带独立供电） | ~¥40 | 🟢 推荐 | D435i 吃 USB 带宽，独立供电 Hub 避免主板 USB 口抢电 |

| 现有硬件（复用） | 用途 |
|:---|:---|
| YDLIDAR X4 | 2D 激光雷达（已集成） |
| ESP32 + MPU6050 | 里程计 + IMU（D435i 内置 IMU 可替代 MPU6050） |
| 电机驱动 + 底盘 | 差分驱动底盘 |
| 电池 | 需确认能否同时供 RPi5 + 深度相机 |

**💰 方案总新增成本：约 ¥1,700（不包含 NVMe SSD）**

---

### 8.2 📈 分级配置方案

#### 🥉 最小可行配置（MVP：纯 2D 激光导航，先跑通）

| 项目 | 选型 | 价格 |
|:---|:---|:---|
| RPi5 8GB | — | ~¥480 |
| Samsung A2 SD 128GB | — | ~¥80 |
| 官方电源 5V/5A | — | ~¥70 |
| 散热片/风扇 | — | ~¥30 |
| **小计** | | **~¥660** |

> ✅ 目标：把现有 PC VM 上的纯激光导航完整跑在 RPi5 上。验证 micro_ros_agent ARM64 编译、Nav2 运行、ESP32 通信。

---

#### 🥈 推荐配置（+深度相机，VoxelLayer 避障）

在 MVP 基础上增加：

| 项目 | 选型 | 价格 |
|:---|:---|:---|
| RealSense D435i | 二手/淘宝 | ~¥1,000-1,200 |
| 有源 USB 3.0 Hub | — | ~¥40 |
| **MVP + 深度相机合计** | | **~¥1,700-1,900** |

> ✅ 目标：3D 避障、更好的障碍物检测（桌腿、悬空物等）。这是**性价比甜点**。

---

#### 🥇 旗舰配置（+NVMe SSD + 可选 TPU）

在推荐配置基础上增加：

| 项目 | 选型 | 价格 |
|:---|:---|:---|
| NVMe SSD 256GB | 如 Kingston NV2 256GB | ~¥120 |
| NVMe 底板 | Pimoroni NVMe BASE / 国产替代 | ~¥60-100 |
| Coral USB TPU | Google Coral（可选） | ~¥400 |
| **推荐配置 + NVMe + TPU 合计** | | **~¥2,280-2,520** |

> ✅ 目标：生产级可靠性（NVMe 杜绝 SD 卡 IO 瓶颈）、YOLO 跟随 30 FPS（TPU 推理）。

---

### 8.3 🛒 具体购买建议

| 序号 | 商品 | 渠道 | 参考价 |
|:---|:---|:---|:---|
| 1 | Raspberry Pi 5 8GB 单板 | 淘宝/京东 Raspberry Pi 官方店 | ¥480 |
| 2 | Samsung PRO Plus 128GB microSD (A2, U3, V30) | 京东自营 | ¥79 |
| 3 | 树莓派 5 官方 USB-C 电源 (5.1V/5A) | 同店购买 | ¥69 |
| 4 | 树莓派 5 官方主动散热器 | 同店购买 | ¥35 |
| 5 | Intel RealSense D435i | 淘宝/闲鱼 | ¥1,000-1,200 |
| 6 | 有源 USB 3.0 Hub (带 5V 供电) | 淘宝 | ¥40 |
| **合计** | | | **~¥1,700** |

> 💡 **省钱技巧**：D435i 闲鱼二手约 ¥600-800（成色好的）；如果不需要内置 IMU，D435（不带 i）更便宜约 ¥800 全新。

---

### 8.4 ⚡ 为什么不推荐某些配置

| 不推荐 | 理由 |
|:---|:---|
| RPi5 4GB 版 | 深度相机点云 + costmap + Nav2 + YOLO 内存峰值可能超 4GB，省 ¥100 冒 OOM 风险不值得 |
| OAK-D Lite 替代 D435i | 深度质量不如 D435i；ROS2 驱动成熟度略低；价格差不多（~¥900） |
| 杂牌 SD 卡 | 非 A2 等级的卡随机 4K 写入只有 0.1-0.5 MB/s，ROS2 日志写多了系统会卡顿 |
| 省掉有源 USB Hub | RPi5 USB 口总供电有限，D435i 峰值功耗 ~1.5W，与激光雷达共用 USB 可能供电不足导致断连 |

---

## 9. 总结

### ✅ 结论：项目可行，中等风险

1. **硬件可行**：RPi5 多核性能 ≈ 当前 2 vCPU VM，4 物理核可更好地隔离节点
2. **软件可行**：ROS2 Humble 全生态支持 ARM64，关键包均为预编译二进制
3. **深度相机可行**：RealSense D435i 在 ARM64 上驱动极度成熟，VoxelLayer 配置已有模板
4. **运行模式匹配**：仅 AMCL + Nav2（不做 SLAM），CPU 利用率约 60-85%，有余量
5. **AI 无影响**：DeepSeek 云端 API，架构无关
6. **跟随模式可用**：YOLOv8n 在 RPi5 上约 3-5 FPS，可接受但非核心路径

### ⚠️ 关键注意事项

- **必须选 8GB 版本**（4GB 在跟随模式 + 深度相机下紧张）
- **SD 卡可行但建议 A2 等级**：先用大容量 A2 SD 卡开发测试完全没问题；长期运行 + 深度相机场景建议升级 NVMe SSD（详见 §1.2）
- **micro_ros_agent 需 ARM64 编译**（这是最大的一次性技术风险）
- **建议先跑 2D 激光导航**，稳定后再加深度相机 VoxelLayer

### 📊 预期成果

迁移完成后，FishBot 将具备：
- 完全自包含的计算平台（RPi5 安装在机器人上）
- 3D 感知能力（深度相机 + VoxelLayer 避障）
- 云端 AI 指令解析（DeepSeek 不变）
- 预建地图导航（无需实时 SLAM）
- 可选人体跟随（YOLO + 深度相机）
- 功耗 <25W 整机（含传感器）
