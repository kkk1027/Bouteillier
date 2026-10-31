-- ============================================================================
-- FishBot 手动建图 Cartographer 2D 配置（IMU 融合版）
--
-- 与 config/fishbot_cartographer.lua 的区别：
--   1) 启用 IMU：订阅 /imu_corrected（ESP32 /imu_raw 经 imu_bias_remover 去偏置）
--   2) tracking_frame = imu_link，因此必须有 base_link -> imu_link 的 TF
--      （manual_mapping.launch.py 默认用 static_transform_publisher 提供）
--   3) 参考实机 stemm_cartographer_exploration/config/stemm_cartographer_2d.lua
--      中与 CPU 开销、回环采样、概率栅格更新相关的参数；
--      但**不照搬**其 Wheeltec 专用帧名（gyro_link / odom_combined）和 12 m 量程。
--
-- 硬件前提：
--   - YDLIDAR（/scan，laser_link，5 Hz，实测有效量程按 8 m 截断）
--   - 编码器里程计 /odom（ESP32），odom2tf 发布 odom -> base_footprint
--   - MPU6050：/imu_raw (100 Hz, imu_link) -> imu_bias_remover -> /imu_corrected
--   - EKF 融合 /odom_ekf + /imu_corrected，仅用于 odom -> base_footprint TF，
--     不参与 Cartographer 的传感器输入
--
-- 注意：
--   - Cartographer 2D 的 IMU 只使用角速度和线加速度（重力对齐），不使用
--     orientation.yaw；因此固件里由陀螺积分的 yaw 漂移不会直接进入建图。
--   - 若临时没有 IMU（例如没有 imu_link TF），请改用 fishbot_cartographer.lua。
-- ============================================================================

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  -- ==================== 坐标帧配置 ====================
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- 使用 IMU 时 tracking_frame 必须是 IMU 所在帧（官方 mir-100-mapping.lua 同做法）
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,          -- odom -> base_footprint 由 odom2tf 提供
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,

  -- ==================== 传感器配置 ====================
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- ==================== 采样率 ====================
  -- 雷达只有 5 Hz，不做抽帧；IMU 100 Hz 全量参与角速度/重力观测
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,

  -- ==================== 发布频率 ====================
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 0.02,      -- 50 Hz，降低 RViz/TF 压力
  trajectory_publish_period_sec = 0.05,
}

-- ==================== 启用 2D SLAM ====================
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2

-- ==================== 2D 轨迹构建器参数 ====================

-- IMU 融合开关（依赖 tracking_frame = imu_link）
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

-- 激光量程：YDLIDAR 驱动 range_min=0.3 / range_max=64，室内按 8 m 截断
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025
TRAJECTORY_BUILDER_2D.min_z = -0.5
TRAJECTORY_BUILDER_2D.max_z = 1.5

-- 自适应体素滤波（与 max_range 对齐，避免远处稀疏点白耗 CPU）
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 8.0
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 8.0

-- 实时相关性扫描匹配：保留以抵抗轮子打滑，但搜索窗口收紧，
-- 避免在低算力平台上抢占 Nav2/其他节点的 CPU（参考实机配置 0.08 m / 6 deg）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.08
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(6.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres 精匹配：默认权重（occupied 1 / translation 10 / rotation 40），
-- 由 IMU 提供旋转先验，比旧的 translation=1 / rotation=1 更稳
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- 运动滤波：参考实机配置，慢速手动推车也能保证足够的位姿节点
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.08
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)

-- 子图内激光帧数
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

-- 概率栅格更新：命中概率略高、清除概率略低，
-- 让真实固定障碍物更"耐久"，又不至于永远擦不掉被移走的物体
local probability_grid_range_data_inserter =
    TRAJECTORY_BUILDER_2D.submaps.range_data_inserter
        .probability_grid_range_data_inserter
probability_grid_range_data_inserter.hit_probability = 0.60
probability_grid_range_data_inserter.miss_probability = 0.495

-- ==================== 位姿图优化参数 ====================
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.sampling_ratio = 0.08
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1

POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 30
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 2

-- 回环：降低全局采样率，只做必要的回环搜索
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 15.0

-- 结束建图时做完整最终优化
POSE_GRAPH.max_num_final_iterations = 200

return options
