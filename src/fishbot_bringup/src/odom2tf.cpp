#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cmath>
#include <string>

// 四元数→偏航角。
//
// ★ 不用 tf2::getYaw()：它内部会调 tf2::fromMsg(Quaternion&)，那是
//   tf2_geometry_msgs 里的符号，本包只依赖 tf2/tf2_ros，链接会报
//       undefined reference to `tf2::fromMsg(...)'
//   这里只有平面运动（two_d_mode），自己算三行就够了。
static double yaw_from_quaternion(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
// ============================================================================
// odom2tf —— /odometry/filtered → odom→base_footprint TF
// ----------------------------------------------------------------------------
// 除了朴素转发，本节点还负责**里程计重置保护**（2026-11-01 实机事故）。
//
// 【事故现场】
//   12:24:12 ~ 12:25:07  odom→base_footprint 稳定累积到 (-12.4, -16.5)，
//                        即 |odom| = 20.6m
//   12:25:08  ★ 一帧之内跳到 (0.047, 0.039)，|odom| 从 20.6m 变成 0.05m
//   之后 AMCL 位姿在整张地图上乱跳，匹配率从 92% 掉到 16~37%，
//   amcl_tracer 记录「跳变=160」。
//
//   根因：ESP32 的 odom 是纯累加、无持久化（Kinematics::update_odom 里
//   就是 odom.x += ...）。芯片一重启（掉电、看门狗、micro-ROS 重建失败
//   50 次后的兜底复位），odom 立刻归零。
//   而 odom→base_footprint 是 TF 树里**唯一**的里程计来源：
//     map --(AMCL)--> odom --(本节点)--> base_footprint
//   odom 一跳，EKF 跟着跳，AMCL 只能用 map→odom 去补偿；而 AMCL 的
//   补偿是概率性的、要好几帧才收敛，中间这段时间整棵 TF 树都是错的。
//
// 【对策：把"重置"吃在 TF 这一层，不让下游看见】
//   维护补偿量 (offset_x, offset_y, offset_yaw)。每帧判断原始 odom 是否
//   发生了**物理上不可能**的跳变：
//       位移/dt > reset_speed_thresh   或   转角/dt > reset_yawrate_thresh
//       （本车最大线速度 0.3 m/s，这里按 3 倍余量取 1.0 m/s）
//   判定为重置时，把补偿量调成「让输出保持在上一帧的位姿」，
//   于是 odom→base_footprint 对下游而言永远连续。
//
//   注意补偿必须**按 yaw 旋转**（见 apply_offset_ 注释），不能直接加。
//
// 【另一个细节：时间戳倒退】
//   ESP32 重启后时间基准可能倒退，倒退的 TF 时间戳会污染 tf2 buffer
//   （表现为 Lookup would require extrapolation）。这里强制单调。
// ============================================================================

class OdomTopic2TF : public rclcpp::Node {
public:
  OdomTopic2TF(std::string name) : Node(name) {
    // ---- 参数 ----
    this->declare_parameter("reset_speed_thresh", 1.0);      // m/s
    this->declare_parameter("reset_yawrate_thresh", 3.15);   // rad/s
    this->declare_parameter("reset_jump_abs", 1.0);          // m，绝对兜底
    this->declare_parameter("use_wall_clock", true);
    this->declare_parameter("enable_reset_guard", true);

    speed_thresh_ = this->get_parameter("reset_speed_thresh").as_double();
    yawrate_thresh_ =
        this->get_parameter("reset_yawrate_thresh").as_double();
    jump_abs_ = this->get_parameter("reset_jump_abs").as_double();
    use_wall_clock_ = this->get_parameter("use_wall_clock").as_bool();
    enable_guard_ = this->get_parameter("enable_reset_guard").as_bool();

    // 订阅 EKF 融合后的里程计（导航模式）或原始 /odom（建图模式）
    odom_subscribe_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", rclcpp::SensorDataQoS(),
        std::bind(&OdomTopic2TF::odom_callback_, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(),
                "odom2tf 已启动：重置保护=%s，速度门限 %.2f m/s / %.2f rad/s",
                enable_guard_ ? "开" : "关", speed_thresh_, yawrate_thresh_);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscribe_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double speed_thresh_{1.0};
  double yawrate_thresh_{3.15};
  double jump_abs_{1.0};
  bool use_wall_clock_{true};
  bool enable_guard_{true};

  // ---- 状态 ----
  bool has_last_{false};
  double last_out_x_{0.0}, last_out_y_{0.0}, last_out_yaw_{0.0};
  double last_raw_x_{0.0}, last_raw_y_{0.0}, last_raw_yaw_{0.0};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  uint64_t resets_{0};
  // 补偿量
  double offset_x_{0.0}, offset_y_{0.0}, offset_yaw_{0.0};

  static double wrap_pi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // 把原始 odom 位姿按当前补偿量变换到"连续"坐标系。
  //
  // ★ 不能简单做 out = raw + offset。odom 坐标系本身有朝向，
  //   平移补偿必须跟着反向旋转，否则车转过 90° 之后补偿方向就错了
  //   （表现：第一次重置保护是对的，越往后越偏）。
  void apply_offset_(double raw_x, double raw_y, double raw_yaw,
                     double &ox, double &oy, double &oyaw) const {
    const double c = std::cos(offset_yaw_);
    const double s = std::sin(offset_yaw_);
    ox = offset_x_ + c * raw_x - s * raw_y;
    oy = offset_y_ + s * raw_x + c * raw_y;
    oyaw = wrap_pi(raw_yaw + offset_yaw_);
  }

  // 由「期望输出」反解补偿量：want = offset + R(dyaw) * raw
  void set_offset_from_(double raw_x, double raw_y, double raw_yaw,
                        double want_x, double want_y, double want_yaw) {
    offset_yaw_ = wrap_pi(want_yaw - raw_yaw);
    const double c = std::cos(offset_yaw_);
    const double s = std::sin(offset_yaw_);
    offset_x_ = want_x - (c * raw_x - s * raw_y);
    offset_y_ = want_y - (s * raw_x + c * raw_y);
  }

  void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const double raw_x = msg->pose.pose.position.x;
    const double raw_y = msg->pose.pose.position.y;
    const double raw_yaw = yaw_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    const rclcpp::Time now = this->now();

    // ---- 1. 判断是否需要补偿 ----
    if (enable_guard_ && has_last_) {
      const double dx = raw_x - last_raw_x_;
      const double dy = raw_y - last_raw_y_;
      const double dist = std::hypot(dx, dy);
      const double dyaw = std::fabs(wrap_pi(raw_yaw - last_raw_yaw_));

      double dt = (now - last_stamp_).seconds();
      if (!std::isfinite(dt) || dt <= 1e-4) {
        dt = 0.033;   // 时间戳异常时按 30Hz 估
      }

      const bool too_fast = (dist / dt) > speed_thresh_;
      const bool spin_too_fast = (dyaw / dt) > yawrate_thresh_;
      const bool huge_jump = dist > jump_abs_;

      if (too_fast || spin_too_fast || huge_jump) {
        // 让这一帧输出**保持在上一次输出**，下游完全无感
        set_offset_from_(raw_x, raw_y, raw_yaw,
                         last_out_x_, last_out_y_, last_out_yaw_);
        ++resets_;
        RCLCPP_WARN(this->get_logger(),
                    "检测到里程计重置（第 %lu 次）：原始 odom (%.2f, %.2f, "
                    "%.1f°) 在 %.3fs 内跳了 %.2fm/%.1f°，已补偿为连续输出",
                    static_cast<unsigned long>(resets_),
                    raw_x, raw_y, raw_yaw * 180.0 / M_PI,
                    dt, dist, dyaw * 180.0 / M_PI);
      }
    }

    // ---- 2. 应用补偿 ----
    double out_x = raw_x, out_y = raw_y, out_yaw = raw_yaw;
    if (enable_guard_) {
      apply_offset_(raw_x, raw_y, raw_yaw, out_x, out_y, out_yaw);
    }

    // ---- 3. 记录状态 ----
    last_raw_x_ = raw_x;
    last_raw_y_ = raw_y;
    last_raw_yaw_ = raw_yaw;
    last_out_x_ = out_x;
    last_out_y_ = out_y;
    last_out_yaw_ = out_yaw;
    has_last_ = true;

    // ---- 4. 发布 ----
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";
    if (use_wall_clock_) {
      // ★ 用当前时刻而不是 msg->header.stamp：ESP32 重启后它自己的时间
      //   基准可能倒退，倒退的 TF 时间戳会污染 tf2 buffer。
      //   本车激光 6Hz、控制 20Hz，这点时间精度损失完全可接受。
      transform.header.stamp = now;
    } else {
      // msg->header.stamp 是 builtin_interfaces::msg::Time，
      // 要和 rclcpp::Time 比较必须先转换（直接比对会找不到 operator<）。
      const rclcpp::Time raw_stamp(msg->header.stamp, this->get_clock()->get_clock_type());
      transform.header.stamp = (raw_stamp < last_stamp_)
                                   ? last_stamp_
                                   : raw_stamp;   // 强制单调
    }
    last_stamp_ = transform.header.stamp;

    transform.transform.translation.x = out_x;
    transform.transform.translation.y = out_y;
    transform.transform.translation.z = 0.0;

    // 欧拉角 → 四元数（只绕 z）
    const double half = out_yaw * 0.5;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = std::sin(half);
    transform.transform.rotation.w = std::cos(half);

    tf_broadcaster_->sendTransform(transform);
  };
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomTopic2TF>("odom2tf");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
