// ros_map_bridge.hpp
#ifndef ROS_MAP_BRIDGE_HPP
#define ROS_MAP_BRIDGE_HPP

#include <QObject>
#include <QThread>
#include <QImage>
#include <QTimer>
#include <memory>
#include <atomic>
#include <QMutex>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/float32.hpp>


// 删除所有前向声明，因为我们已经包含了完整的头文件
// 不再需要这些前向声明！

class RosMapBridge : public QObject
{
    Q_OBJECT

public:
    explicit RosMapBridge(QObject *parent = nullptr);
    ~RosMapBridge();
    void requestMap();
    // 初始化ROS2节点
    bool initROS2(const QString& node_name = "qt_map_bridge");
    void forceCleanup();
    // 关闭ROS2节点
    void shutdownROS2();

    // 设置ROS2域名ID
    void setDomainId(int domain_id) { domain_id_ = domain_id; }

    // 设置订阅话题
    void setMapTopic(const QString& topic) { map_topic_ = topic; }
    void setRobotPoseTopic(const QString& topic) { robot_pose_topic_ = topic; }
    void setGoalPoseTopic(const QString& topic) { goal_pose_topic_ = topic; }
    void setTrajectoryTopic(const QString& topic) { trajectory_topic_ = topic; }

    // 设置TF坐标系
    void setMapFrame(const QString& frame) { map_frame_ = frame; }
    void setRobotFrame(const QString& frame) { robot_frame_ = frame; }

    // 检查状态
    bool isInitialized() const { return ros_initialized_.load(); }
    bool isMapReceived() const { return map_received_.load(); }
    bool isRobotPoseAvailable() const { return robot_pose_available_.load(); }

signals:
    // 地图数据信号
    void mapReceived(const QImage& map_image, float resolution,
                     float origin_x, float origin_y);

    // 机器人位姿信号
    void robotPoseUpdated(float x, float y, float theta);

    // 目标点信号
    void goalPoseUpdated(float x, float y, float theta);

    // 轨迹点信号
    void trajectoryPointAdded(float x, float y);
    void batteryUpdated(float percent);
    // 状态信号
    void statusChanged(const QString& status);
    void errorOccurred(const QString& error);

public slots:
    // 发送目标点（用于UI设置目标）
    void sendGoalPose(float x, float y, float theta);
    // 开始/停止订阅
    void startSubscriptions();
    void stopSubscriptions();

private:
    // ROS2线程工作函数
    void rosThreadWorker();

    // 初始化ROS2节点（在ROS线程中调用）
    bool initROS2Node();

    // 订阅回调 - 使用正确的类型
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void robotPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // TF查询
    bool getRobotPoseFromTF(float& x, float& y, float& theta);

    // 地图数据转换
    QImage occupancyGridToQImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid);

    // 检查ROS2环境
    bool checkROS2Environment();

    // ROS2相关成员
    std::shared_ptr<rclcpp::Node> ros_node_;
    std::shared_ptr<rclcpp::Context> ros_context_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 状态标志
    std::atomic<bool> ros_initialized_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> map_received_{false};
    std::atomic<bool> robot_pose_available_{false};
    std::atomic<bool> shutdown_requested_{false};

    // ROS2线程
    QThread* ros_thread_;

    // 定时器
    QTimer* tf_timer_;
    QTimer* battery_check_timer_ = nullptr;

    // 话题和帧配置
    QString map_topic_ = "/map";
    QString robot_pose_topic_ = "/amcl_pose";
    QString goal_pose_topic_ = "/goal_pose";
    QString trajectory_topic_ = "/plan";
    QString map_frame_ = "map";
    QString robot_frame_ = "base_footprint";
    QString odom_frame_ = "odom";

    // 域名ID
    int domain_id_ = 0;

    // 缓存数据
    QImage current_map_;
    float map_resolution_ = 0.0f;
    float map_origin_x_ = 0.0f;
    float map_origin_y_ = 0.0f;
    float current_battery_ = -1.0;  // -1 表示未获取到数据

    // 机器人位姿缓存
    struct RobotPose {
        float x = 0.0f;
        float y = 0.0f;
        float theta = 0.0f;
        qint64 timestamp = 0;
    };
    RobotPose current_robot_pose_;

    // 轨迹点缓存
    struct TrajectoryPoint {
        float x = 0.0f;
        float y = 0.0f;
        qint64 timestamp = 0;
    };
    QVector<TrajectoryPoint> trajectory_points_;

    // 互斥锁（用于线程安全）
    mutable QMutex data_mutex_;

    // 订阅频率控制
    int tf_update_rate_ = 10;  // Hz
    qint64 last_tf_query_time_ = 0;

    // 订阅器
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_subscription_;

    // 发布器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
};

#endif // ROS_MAP_BRIDGE_HPP
