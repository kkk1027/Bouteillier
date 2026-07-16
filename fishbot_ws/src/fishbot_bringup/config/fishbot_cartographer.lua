-- ============================================================================
-- FishBot Cartographer 2D SLAM 配置文件
--
-- 适配硬件：
--   - YDLIDAR X2/X4 (2D 激光雷达)
--   - 编码器里程计 (odom2tf 节点)
--   - MPU6050 IMU (通过 EKF 融合，无独立 imu_link)
--   - URDF: base_footprint → base_link → laser_link
--
-- 参考: turtlebot3_lds_2d.lua (TurtleBot3 官方配置)
-- ============================================================================

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  -- ==================== 坐标帧配置 ====================
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",                          -- 全局地图坐标系
  tracking_frame = "base_link",               -- 跟踪帧：激光雷达装在 base_link 上
  published_frame = "odom",                   -- Cartographer 发布的里程计帧
  odom_frame = "odom",                        -- 里程计坐标系（外部的 odom2tf 提供）
  provide_odom_frame = false,                 -- 不发布 odom TF（由 odom2tf 负责）
  publish_frame_projected_to_2d = true,       -- 将位姿投影到 2D 平面

  -- ==================== 传感器配置 ====================
  use_odometry = true,                        -- 使用编码器里程计辅助扫描匹配
  use_nav_sat = false,                        -- 不使用 GPS
  use_landmarks = false,                      -- 不使用路标
  num_laser_scans = 1,                        -- 单束激光扫描
  num_multi_echo_laser_scans = 0,             -- 无多回波激光
  num_subdivisions_per_laser_scan = 1,        -- 每帧扫描不细分
  num_point_clouds = 0,                       -- 不使用点云

  -- ==================== 采样率 ====================
  rangefinder_sampling_ratio = 1.,            -- 激光测距全采样
  odometry_sampling_ratio = 1.,               -- 里程计全采样
  fixed_frame_pose_sampling_ratio = 1.,       -- 固定帧全采样
  imu_sampling_ratio = 1.,                    -- IMU 全采样（虽然 use_imu_data=false）
  landmarks_sampling_ratio = 1.,              -- 路标全采样

  -- ==================== 发布频率 ====================
  lookup_transform_timeout_sec = 0.2,         -- TF 查找超时
  submap_publish_period_sec = 0.3,            -- 子图发布间隔
  pose_publish_period_sec = 5e-3,             -- 位姿发布间隔 (200Hz)
  trajectory_publish_period_sec = 30e-3,      -- 轨迹发布间隔 (~33Hz)
}

-- ==================== 启用 2D SLAM ====================
MAP_BUILDER.use_trajectory_builder_2d = true

-- ==================== 2D 轨迹构建器参数 ====================

-- 激光范围
TRAJECTORY_BUILDER_2D.min_range = 0.12        -- YDLIDAR 最小有效距离 (m)
TRAJECTORY_BUILDER_2D.max_range = 8.0         -- YDLIDAR 最大有效距离 (m)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0   -- 无数据时的默认射线长度

-- 使用实时相关性扫描匹配（提高局部精度，增加 CPU 开销）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 实时相关性扫描匹配参数
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2     -- 线性搜索窗口 (m)，增大以更好匹配回环
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)  -- 角度搜索窗口 (度)，增大以更好匹配回环
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres 扫描匹配（精匹配）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.    -- 平移权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.       -- 旋转权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 15  -- 增加迭代次数提高扫描匹配精度
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- 运动滤波：只有超过阈值才插入新位姿
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)       -- ~5.7°
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1               -- 10cm

-- 自适应体素滤波（降采样激光点）
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 8.0

-- 环回检测的体素滤波
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 8.0

-- 体素滤波类型
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

-- 是否使用 IMU 数据（FishBot 无独立 imu_link，设为 false）
-- 如果后续在 URDF 中添加了 imu_link，可以改为 true
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 如果不用 IMU，需要怎么处理重力对齐
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- ==================== 位姿图优化参数 ====================

-- 回环约束的最小匹配分数
POSE_GRAPH.constraint_builder.min_score = 0.55              -- YDLIDAR点云稀疏，降低阈值使更多回环被接受
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6  -- 全局定位也同步降低

-- 约束构建器参数
POSE_GRAPH.constraint_builder.log_matches = true

-- 回环检测的采样率
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5              -- 提高回环候选帧采样率

-- 优化参数
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 3e5
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

-- 优化频率：每 100 个节点优化一次位姿图（约14秒一次）
-- 原值20（约3秒一次）导致map→odom频繁调整，base_link在RViz中抖动
-- 设为0表示只优化一次（建图结束时）；设为正数在过程中持续优化
POSE_GRAPH.optimize_every_n_nodes = 100    -- 降低优化频率，减少抖动，回环依然有效

-- 全局定位的采样参数
POSE_GRAPH.global_sampling_ratio = 0.01               -- 从 0.3% 提高到 1%，增加全局回环检测频率

-- ==================== 输出配置 ====================
POSE_GRAPH.max_num_final_iterations = 200     -- 最终优化的迭代次数

return options
