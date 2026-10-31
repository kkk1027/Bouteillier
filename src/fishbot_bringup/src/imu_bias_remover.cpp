#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <algorithm>
#include <deque>
#include <vector>

class ImuBiasRemover : public rclcpp::Node
{
public:
  ImuBiasRemover(std::string name) : Node(name)
  {
    // ========== 可配置参数 ==========
    // 静止判定阈值。
    //
    // 原值 wz 阈值 0.02 rad/s 有一个致命问题：底层 /odom 来自编码器差分
    // 积分，静止时 ±1 tick 的量化噪声就能产生 0.025~0.05 rad/s 的虚假
    // 角速度（dt=10ms、轮距 212.3mm、系数 105.805），**永远超过 0.02**。
    // 结果本节点永远认为机器人在运动，wz_samples_ 永远采不到样本，
    // 陀螺零偏标定永远无法完成。
    //
    // 根治办法是在固件里给 odom 积分加死区（2026-10-31 已改
    // Kinematics.cpp：静止时不积分，twist.angular.z 直接输出 0）。
    // 这里把阈值放宽到 0.05 作为第二道保险：万一固件没更新、或桥接丢帧
    // 导致偶发跳变，标定依然能完成。
    // 代价：0.05 rad/s ≈ 2.9 deg/s 以下的真实转动不会被当成"运动"，
    // 而这么慢的转动本来也不需要实时偏置跟踪。
    this->declare_parameter<double>("stationary_vx_thresh", 0.02);
    this->declare_parameter<double>("stationary_wz_thresh", 0.05);
    this->declare_parameter<int>("calib_samples", 100);
    this->declare_parameter<double>("calib_duration", 3.0);

    stationary_vx_thresh_ = this->get_parameter("stationary_vx_thresh").as_double();
    stationary_wz_thresh_ = this->get_parameter("stationary_wz_thresh").as_double();
    calib_samples_ = this->get_parameter("calib_samples").as_int();
    calib_duration_ns_ = static_cast<int64_t>(
        this->get_parameter("calib_duration").as_double() * 1e9);

    // ========== 发布器 ==========
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_corrected", 10);

    // ========== 订阅器 ==========
    // 订阅 ESP32 原始 IMU 数据（含陀螺偏置）
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_raw", rclcpp::SensorDataQoS(),
        std::bind(&ImuBiasRemover::imu_callback_, this, std::placeholders::_1));

    // 订阅编码器里程计，用于检测静止状态
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        std::bind(&ImuBiasRemover::odom_callback_, this, std::placeholders::_1));

    // ========== 初始状态 ==========
    wz_bias_ = 0.0;        // 初始偏置为0（无修正）
    bias_initialized_ = false;
    is_stationary_ = false;

    RCLCPP_INFO(this->get_logger(), "IMU 偏置校准节点已启动");
    RCLCPP_INFO(this->get_logger(), "机器人静止后将自动校准陀螺Z轴偏置");
  }

private:
  // ---- 发布器/订阅器 ----
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // ---- 参数 ----
  double stationary_vx_thresh_;   // 静止线速度阈值 (m/s)
  double stationary_wz_thresh_;   // 静止角速度阈值 (rad/s)
  int calib_samples_;             // 标定所需最小样本数
  int64_t calib_duration_ns_;     // 标定所需最小时长 (ns)

  // ---- 状态 ----
  double wz_bias_;                        // 当前陀螺Z轴偏置估计值 (rad/s)
  bool bias_initialized_;                 // 偏置是否已完成初始标定
  bool is_stationary_;                    // 当前是否处于静止状态
  std::deque<double> wz_samples_;         // 静止期间的 wz 采样缓冲
  rclcpp::Time stationary_start_time_;    // 本次静止期开始时间

  // ============================================================
  // 里程计回调 — 检测机器人是否静止
  // ============================================================
  void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double vx = msg->twist.twist.linear.x;
    double wz = msg->twist.twist.angular.z;

    bool now_stationary = (std::abs(vx) < stationary_vx_thresh_ &&
                           std::abs(wz) < stationary_wz_thresh_);

    if (now_stationary) {
      if (!is_stationary_) {
        // 刚进入静止状态 → 重置采样缓冲
        is_stationary_ = true;
        stationary_start_time_ = this->now();
        wz_samples_.clear();
        RCLCPP_DEBUG(this->get_logger(),
            "检测到静止，开始采集陀螺偏置样本...");
      }
    } else {
      if (is_stationary_) {
        RCLCPP_DEBUG(this->get_logger(),
            "机器人运动，当前偏置 = %.6f rad/s", wz_bias_);
      }
      is_stationary_ = false;
    }
  }

  // ============================================================
  // IMU 回调 — 去除偏置 + 发布修正数据
  // ============================================================
  void imu_callback_(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double raw_wz = msg->angular_velocity.z;

    // ---------- 静止期间：采集样本用于偏置估算 ----------
    if (is_stationary_) {
      wz_samples_.push_back(raw_wz);

      rclcpp::Time now = this->now();
      int64_t elapsed_ns = (now - stationary_start_time_).nanoseconds();

      if (!bias_initialized_ &&
          static_cast<int>(wz_samples_.size()) >= calib_samples_ &&
          elapsed_ns >= calib_duration_ns_)
      {
        // 初始标定：有足够样本后一次性计算偏置
        compute_bias_();
        bias_initialized_ = true;
        RCLCPP_INFO(this->get_logger(),
            "✅ 陀螺偏置初始标定完成: wz_bias = %.6f rad/s (%d 个样本)",
            wz_bias_, static_cast<int>(wz_samples_.size()));
      }
      else if (bias_initialized_ &&
               static_cast<int>(wz_samples_.size()) >= calib_samples_ / 2 &&
               elapsed_ns >= calib_duration_ns_ / 2)
      {
        // 运行时重标定：跟踪温漂，用较少样本更新偏置
        compute_bias_();
        RCLCPP_DEBUG(this->get_logger(),
            "陀螺偏置已更新: %.6f rad/s", wz_bias_);
      }
    }

    // ---------- 对所有数据应用偏置修正 ----------
    double corrected_wz = raw_wz - wz_bias_;

    // 构造修正后的 IMU 消息
    auto corrected_msg = std::make_shared<sensor_msgs::msg::Imu>();

    // 保持原始时间戳和 frame_id
    corrected_msg->header = msg->header;

    // 透传 orientation（EKF 不使用，但保留原始数据)
    corrected_msg->orientation = msg->orientation;
    corrected_msg->orientation_covariance = msg->orientation_covariance;

    // 透传加速度
    corrected_msg->linear_acceleration = msg->linear_acceleration;
    corrected_msg->linear_acceleration_covariance = msg->linear_acceleration_covariance;

    // 角速度：只修正 wz（差速机器人的偏置主要在Z轴）
    corrected_msg->angular_velocity = msg->angular_velocity;
    corrected_msg->angular_velocity.z = corrected_wz;
    corrected_msg->angular_velocity_covariance = msg->angular_velocity_covariance;

    imu_pub_->publish(*corrected_msg);
  }

  // ============================================================
  // 偏置估算：使用中位数（抗离群值）
  // ============================================================
  void compute_bias_()
  {
    if (wz_samples_.empty()) return;

    std::vector<double> sorted(wz_samples_.begin(), wz_samples_.end());
    std::sort(sorted.begin(), sorted.end());

    size_t n = sorted.size();
    if (n % 2 == 0) {
      wz_bias_ = (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0;
    } else {
      wz_bias_ = sorted[n / 2];
    }

    // 清空缓冲，避免重复使用过期数据
    wz_samples_.clear();
  }
};

// ============================================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuBiasRemover>("imu_bias_remover");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
