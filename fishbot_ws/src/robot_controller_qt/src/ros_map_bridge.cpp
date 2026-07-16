// ros_map_bridge.cpp
#include "ros_map_bridge.hpp"

#include <QDebug>
#include <QThread>
#include <QTimer>
#include <QMutexLocker>
#include <QCoreApplication>
#include <QDir>
#include <QDateTime>
#include <cmath>

// ROS2 包含
#ifdef HAVE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// 更新这个包含，使用新的头文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 注意：是 .hpp 不是 .h
#endif

using namespace std::chrono_literals;

RosMapBridge::RosMapBridge(QObject *parent)
    : QObject(parent)
    , ros_thread_(nullptr)
    , tf_timer_(nullptr)
    , ros_initialized_(false)
    , running_(false)
    , map_received_(false)
    , robot_pose_available_(false)
    , shutdown_requested_(false)
    , map_resolution_(0.0f)
    , map_origin_x_(0.0f)
    , map_origin_y_(0.0f)
    , domain_id_(0)
    , tf_update_rate_(10)
    , last_tf_query_time_(0)
{
    // 设置默认话题
    map_topic_ = "/map";
    robot_pose_topic_ = "/amcl_pose";
    goal_pose_topic_ = "/goal_pose";
    trajectory_topic_ = "/plan";

    // 设置坐标系
    map_frame_ = "map";
    robot_frame_ = "base_footprint";
    odom_frame_ = "odom";

    qDebug() << "RosMapBridge创建";
}

RosMapBridge::~RosMapBridge()
{
    shutdownROS2();
    forceCleanup();
}

bool RosMapBridge::checkROS2Environment()
{
    // 检查环境变量
    QString rosDistro = qEnvironmentVariable("ROS_DISTRO");
    if (rosDistro.isEmpty()) {
        qWarning() << "ROS_DISTRO not set";
        return false;
    }

    // 检查ROS2安装
    QString rosPath = "/opt/ros/" + rosDistro;
    if (!QDir(rosPath).exists()) {
        qWarning() << "ROS2 installation not found at:" << rosPath;
        return false;
    }

    // 检查工作空间
    QString workspacePath = qEnvironmentVariable("FISHBOT_WS", "/home/ke/chapt9_update/fishbot_ws");
    if (!QDir(workspacePath).exists()) {
        qWarning() << "Workspace not found at:" << workspacePath;
        return false;
    }

    return true;
}
void RosMapBridge::forceCleanup()
{
    qDebug() << "=== 强制清理ROS2资源 ===";

    // 先调用正常关闭
    shutdownROS2();

    // 等待一小段时间确保资源释放
    QThread::msleep(500);

    // 额外清理：如果有残留的线程，确保被清理
    if (ros_thread_) {
        if (ros_thread_->isRunning()) {
            qWarning() << "线程仍在运行，强制终止";
            ros_thread_->terminate();
            ros_thread_->wait(1000);
        }
        delete ros_thread_;
        ros_thread_ = nullptr;
    }

    // 确保所有智能指针都被释放
    map_subscription_.reset();
    robot_pose_subscription_.reset();
    goal_subscription_.reset();
    goal_publisher_.reset();
    tf_buffer_.reset();
    tf_listener_.reset();
    ros_node_.reset();

    qDebug() << "ROS2强制清理完成";
    emit statusChanged("ROS2资源已强制清理");
}
bool RosMapBridge::initROS2(const QString& node_name)
{
    Q_UNUSED(node_name);

    if (ros_initialized_.load()) {
        qDebug() << "ROS2 already initialized";
        return true;
    }

    // 检查环境
    if (!checkROS2Environment()) {
        emit errorOccurred("ROS2环境检查失败");
        return false;
    }

    // 设置环境变量
    qputenv("ROS_DOMAIN_ID", QByteArray::number(domain_id_));
    qputenv("ROS_LOCALHOST_ONLY", "0");  // 允许跨机器通信

        qDebug() << "设置ROS_DOMAIN_ID =" << domain_id_;
        qDebug() << "当前ROS环境:";
        qDebug() << "  ROS_DISTRO:" << qgetenv("ROS_DISTRO");
        qDebug() << "  ROS_DOMAIN_ID:" << qgetenv("ROS_DOMAIN_ID");
        qDebug() << "  ROS_VERSION:" << qgetenv("ROS_VERSION");
    // 创建ROS线程
    ros_thread_ = new QThread(this);

    // 移动到ROS线程
    this->moveToThread(ros_thread_);

    // 连接线程信号
    connect(ros_thread_, &QThread::started, this, &RosMapBridge::rosThreadWorker);
    connect(ros_thread_, &QThread::finished, ros_thread_, &QThread::deleteLater);

    // 启动线程
    ros_thread_->start();

    // 等待初始化
    for (int i = 0; i < 50; ++i) {  // 最多等待5秒
        if (ros_initialized_.load()) {
            break;
        }
        QThread::msleep(100);
    }

    if (!ros_initialized_.load()) {
        emit errorOccurred("ROS2节点初始化超时");
        return false;
    }

    emit statusChanged("ROS2节点初始化成功");
    return true;
}

bool RosMapBridge::initROS2Node()
{
#ifdef HAVE_ROS2
    try {
        // 初始化ROS2上下文
        rclcpp::init(0, nullptr);

        // 创建节点
        ros_node_ = std::make_shared<rclcpp::Node>("qt_map_bridge");

        // 创建TF buffer和listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, ros_node_);

        qDebug() << "ROS2节点创建成功:" << ros_node_->get_name();
        return true;

    } catch (const std::exception& e) {
        qCritical() << "初始化ROS2节点失败:" << e.what();
        return false;
    }
#else
    qWarning() << "ROS2未启用 (HAVE_ROS2未定义)";
    return false;
#endif
}

void RosMapBridge::rosThreadWorker()
{
#ifdef HAVE_ROS2
    try {
        if (!initROS2Node()) {
            emit errorOccurred("ROS2节点初始化失败");
            return;
        }

        ros_initialized_.store(true);
        emit statusChanged("ROS2节点运行中");

        auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(ros_node_);

        running_.store(true);
        shutdown_requested_.store(false);

        // 修改：增加更频繁的退出检查
        while (rclcpp::ok() && running_.load() && !shutdown_requested_.load()) {
            executor->spin_once(100ms);
            // 每100ms检查一次退出标志
        }

        qDebug() << "ROS线程退出循环";

        // 清理
        executor->remove_node(ros_node_);

        // 在退出前重置节点
        ros_node_.reset();
        tf_listener_.reset();
        tf_buffer_.reset();

        ros_initialized_.store(false);
        running_.store(false);

        emit statusChanged("ROS2节点已停止");
        qDebug() << "ROS线程工作器退出";

    } catch (const std::exception& e) {
        qCritical() << "ROS线程异常:" << e.what();
        emit errorOccurred(QString("ROS线程异常: %1").arg(e.what()));
        ros_initialized_.store(false);
        running_.store(false);
    }
#else
    qWarning() << "ROS2未启用，线程工作器不会运行";
#endif
}

void RosMapBridge::shutdownROS2()
{
    qDebug() << "=== 开始关闭ROS2 ===";

    if (!ros_initialized_.load()) {
        qDebug() << "ROS2未初始化，无需关闭";
        return;
    }

    // 停止定时器
    if (tf_timer_) {
        tf_timer_->stop();
        delete tf_timer_;
        tf_timer_ = nullptr;
    }

    // 停止订阅（先取消订阅）
    map_subscription_.reset();
    robot_pose_subscription_.reset();
    goal_subscription_.reset();
    goal_publisher_.reset();

    // 请求线程停止
    shutdown_requested_.store(true);
    running_.store(false);

    // 等待ROS线程结束
    if (ros_thread_ && ros_thread_->isRunning()) {
        qDebug() << "等待ROS线程退出...";

        // 先尝试优雅退出
        ros_thread_->quit();

        // 等待最多3秒
        for (int i = 0; i < 30; i++) {
            if (!ros_thread_->isRunning()) break;
            QThread::msleep(100);
        }

        // 如果还没退出，强制终止
        if (ros_thread_->isRunning()) {
            qWarning() << "ROS线程未响应，强制终止";
            ros_thread_->terminate();
            ros_thread_->wait(1000);
        }
    }

    // 清理TF相关资源
    tf_buffer_.reset();
    tf_listener_.reset();

    // 清理节点（必须在 shutdown 之前）
    ros_node_.reset();

    // 尝试关闭rclcpp（但注意：这可能已经由线程内部处理了）
    try {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
            qDebug() << "rclcpp::shutdown() 已调用";
        }
    } catch (const std::exception& e) {
        qWarning() << "rclcpp::shutdown() 异常:" << e.what();
    }

    // 重置状态标志
    ros_initialized_.store(false);
    map_received_.store(false);
    robot_pose_available_.store(false);
    shutdown_requested_.store(false);
    running_.store(false);

    qDebug() << "ROS2关闭完成";
    emit statusChanged("ROS2已关闭");
}

void RosMapBridge::startSubscriptions()
{
#ifdef HAVE_ROS2
    if (!ros_initialized_.load() || !ros_node_) {
        emit errorOccurred("ROS2节点未初始化，无法订阅");
        return;
    }

    qDebug() << "开始订阅话题...";
    qDebug() << "地图话题:" << map_topic_;
    qDebug() << "位姿话题:" << robot_pose_topic_;

    QMutexLocker locker(&data_mutex_);

    try {
        // 1. 打印所有可用话题用于调试
        auto topics = ros_node_->get_topic_names_and_types();
        qDebug() << "=== 可用ROS2话题 ===";
        for (const auto& [topic_name, types] : topics) {
            QString qname = QString::fromStdString(topic_name);
            if (qname.contains("map") || qname.contains("pose") || qname.contains("costmap")) {
                qDebug() << "话题:" << qname << "类型:" << QString::fromStdString(types[0]);
            }
        }
        qDebug() << "===================";

        // 2. 创建地图订阅 - 使用正确的QoS设置
        // 地图通常使用 TransientLocal 持久性
        rclcpp::QoS map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .transient_local()
            .reliable()
            .durability_volatile();

        qDebug() << "创建地图订阅，QoS: TransientLocal, Reliable";

        auto map_callback = [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            qDebug() << "mapCallback被调用! 地图大小:"
                     << msg->info.width << "x" << msg->info.height;
            this->mapCallback(msg);
        };

        map_subscription_ = ros_node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_.toStdString(),
            map_qos,
            map_callback
        );
        battery_subscription_ = ros_node_->create_subscription<std_msgs::msg::Float32>(
            "/battery_percent",
            10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                current_battery_ = msg->data;
                qDebug() << "收到电池电量:" << current_battery_ << "%";
                emit batteryUpdated(current_battery_);
            }
        );
        qDebug() << "地图订阅创建成功，主题:" << map_topic_;

        // 3. 创建机器人位姿订阅
        rclcpp::QoS pose_qos(10);
        pose_qos.reliable();

        auto robot_pose_callback = [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            qDebug() << "robotPoseCallback被调用! 位置:("
                     << msg->pose.pose.position.x << ","
                     << msg->pose.pose.position.y << ")";
            this->robotPoseCallback(msg);
        };

        robot_pose_subscription_ = ros_node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            robot_pose_topic_.toStdString(),
            pose_qos,
            robot_pose_callback
        );

        qDebug() << "位姿订阅创建成功，主题:" << robot_pose_topic_;
        // 5. 创建电池电量订阅
        rclcpp::QoS battery_qos(10);
        battery_qos.reliable();

        auto battery_callback = [this](const std_msgs::msg::Float32::SharedPtr msg) {
            qDebug() << "收到电池电量:" << msg->data << "%";
            current_battery_ = msg->data;
            emit batteryUpdated(current_battery_);
        };

        battery_subscription_ = ros_node_->create_subscription<std_msgs::msg::Float32>(
            "/battery_percent",
            battery_qos,
            battery_callback
        );

        qDebug() << "电池电量订阅创建成功，主题: /battery_percent";
        // 4. 创建TF查询定时器
        if (!tf_timer_) {
            tf_timer_ = new QTimer();
            tf_timer_->moveToThread(QThread::currentThread());
            connect(tf_timer_, &QTimer::timeout, this, [this]() {
                float x, y, theta;
                if (this->getRobotPoseFromTF(x, y, theta)) {
                    qDebug() << "TF查询到机器人位姿:(" << x << "," << y << ") θ=" << theta;
                    QMetaObject::invokeMethod(this, [this, x, y, theta]() {
                        emit this->robotPoseUpdated(x, y, theta);
                    }, Qt::QueuedConnection);
                } else {
                    qDebug() << "TF查询失败";
                }
            });

            QMetaObject::invokeMethod(tf_timer_, [this]() {
                tf_timer_->start(200);  // 5 Hz 更新频率
            }, Qt::QueuedConnection);
        }

        emit statusChanged("已开始订阅话题");
        qDebug() << "所有订阅已成功创建";

        // 5. 发送测试信号确认连接
        QTimer::singleShot(1000, this, [this]() {
            qDebug() << "发送测试信号...";
            emit statusChanged("订阅测试信号");
        });

    } catch (const std::exception& e) {
        qCritical() << "创建订阅失败:" << e.what();
        emit errorOccurred(QString("创建订阅失败: %1").arg(e.what()));
    }
#else
    qWarning() << "ROS2未启用，无法开始订阅";
    emit errorOccurred("ROS2未启用");
#endif
}void RosMapBridge::stopSubscriptions()
{
    if (tf_timer_) {
        tf_timer_->stop();
        delete tf_timer_;
        tf_timer_ = nullptr;
    }

    emit statusChanged("已停止订阅话题");
}

// 关键修正点：使用与头文件完全一致的函数签名
void RosMapBridge::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
#ifdef HAVE_ROS2
    if (!msg) {
        qWarning() << "mapCallback: 收到空消息";
        return;
    }

    qDebug() << "=== 地图回调开始 ===";
    qDebug() << "地图宽度:" << msg->info.width;
    qDebug() << "地图高度:" << msg->info.height;
    qDebug() << "分辨率:" << msg->info.resolution;
    qDebug() << "原点:(" << msg->info.origin.position.x
             << "," << msg->info.origin.position.y << ")";
    qDebug() << "数据大小:" << msg->data.size();

    QMutexLocker locker(&data_mutex_);

    try {
        qDebug() << "开始转换地图到QImage...";
        QImage map_image = occupancyGridToQImage(msg);

        if (map_image.isNull()) {
            qWarning() << "地图转换失败: 生成空图像";
            return;
        }

        qDebug() << "QImage创建成功，大小:" << map_image.size();

        current_map_ = map_image;
        map_resolution_ = msg->info.resolution;
        map_origin_x_ = msg->info.origin.position.x;
        map_origin_y_ = msg->info.origin.position.y;

        map_received_.store(true);

        qDebug() << "准备发出地图信号...";
        qDebug() << "当前线程:" << QThread::currentThread();
        qDebug() << "对象线程:" << this->thread();
        if (map_image.isNull()) {
                qWarning() << "map_image 为空，无法发送信号";
                return;
            }
        // 关键修复：直接发出信号（不再使用 invokeMethod）
        // 因为信号槽连接已经使用了 Qt::QueuedConnection
        qDebug() << "直接发出 mapReceived 信号...";

        // 发出信号 - 这将自动排队到主线程
        emit mapReceived(map_image, map_resolution_, map_origin_x_, map_origin_y_);

        // 同时发出状态信号
        emit statusChanged(QString("地图已接收 (%1x%2)")
            .arg(map_image.width())
            .arg(map_image.height()));

        qDebug() << "地图信号已发出";

        qDebug() << "=== 地图回调结束 ===";

    } catch (const std::exception& e) {
        qCritical() << "处理地图回调时出错:" << e.what();
    }
#else
    Q_UNUSED(msg);
    qWarning() << "ROS2未启用，无法处理地图回调";
#endif
}QImage RosMapBridge::occupancyGridToQImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
{
#ifdef HAVE_ROS2
    if (!grid || grid->data.empty()) {
        qWarning() << "occupancyGridToQImage: 网格数据为空";
        return QImage();
    }

    int width = grid->info.width;
    int height = grid->info.height;

    if (width <= 0 || height <= 0) {
        qWarning() << "occupancyGridToQImage: 无效的尺寸:" << width << "x" << height;
        return QImage();
    }

    qDebug() << "occupancyGridToQImage: 转换" << width << "x" << height
             << "网格，数据大小:" << grid->data.size();

    // 创建QImage - 使用 ARGB32 格式确保兼容性
    QImage image(width, height, QImage::Format_ARGB32);

    if (image.isNull()) {
        qCritical() << "occupancyGridToQImage: 无法创建QImage";
        return QImage();
    }

    // 填充像素数据
    for (int y = 0; y < height; ++y) {
        QRgb* scanLine = reinterpret_cast<QRgb*>(image.scanLine(y));
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;

            // 确保索引有效
            if (index >= static_cast<int>(grid->data.size())) {
                qWarning() << "索引越界:" << index << ">=" << grid->data.size();
                scanLine[x] = qRgb(127, 127, 127);  // 灰色表示错误
                continue;
            }

            int8_t value = grid->data[index];

            // 转换：-1=未知=灰色，0=空闲=白色，100=占用=黑色
            QRgb pixel_color;
            if (value == -1) {
                pixel_color = qRgb(127, 127, 127);  // 灰色表示未知区域
            } else if (value == 0) {
                pixel_color = qRgb(255, 255, 255);  // 白色表示空闲区域
            } else if (value == 100) {
                pixel_color = qRgb(0, 0, 0);        // 黑色表示占用区域
            } else {
                // 部分占用的区域使用灰度
                int gray = 255 - (value * 255 / 100);
                pixel_color = qRgb(gray, gray, gray);
            }

            scanLine[x] = pixel_color;
        }
    }

    qDebug() << "occupancyGridToQImage: 转换完成，图像格式:" << image.format();
    return image;
#else
    Q_UNUSED(grid);
    qWarning() << "ROS2未启用，无法转换地图";
    return QImage();
#endif
}
// 关键修正点：使用与头文件完全一致的函数签名
void RosMapBridge::robotPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
#ifdef HAVE_ROS2
    if (!msg) {
        qWarning() << "robotPoseCallback: 收到空消息";
        return;
    }

    QMutexLocker locker(&data_mutex_);

    try {
        // 更新机器人位姿
        current_robot_pose_.x = msg->pose.pose.position.x;
        current_robot_pose_.y = msg->pose.pose.position.y;

        // 从四元数计算偏航角
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_robot_pose_.theta = static_cast<float>(yaw);
        current_robot_pose_.timestamp = QDateTime::currentMSecsSinceEpoch();

        robot_pose_available_.store(true);

        float x = current_robot_pose_.x;
        float y = current_robot_pose_.y;
        float theta = current_robot_pose_.theta;

        qDebug() << "[ROS2] 准备发出机器人位姿信号: (" << x << ", " << y << "), θ=" << theta;
        qDebug() << "[ROS2] 当前线程: " << QThread::currentThread();

        // 直接发出信号 - 确保信号被发出
        emit robotPoseUpdated(x, y, theta);

        qDebug() << "[ROS2] 信号已发出";

    } catch (const std::exception& e) {
        qCritical() << "处理机器人位姿回调时出错:" << e.what();
    }
#else
    Q_UNUSED(msg);
    qWarning() << "ROS2未启用，无法处理位姿回调";
#endif
}
// 关键修正点：使用与头文件完全一致的函数签名
void RosMapBridge::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
#ifdef HAVE_ROS2
    if (!msg) {
        return;
    }

    QMutexLocker locker(&data_mutex_);

    try {
        float x = msg->pose.position.x;
        float y = msg->pose.position.y;

        // 从四元数计算偏航角
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        float theta = static_cast<float>(yaw);

        // 发送信号
        QMetaObject::invokeMethod(this, [this, x, y, theta]() {
            emit this->goalPoseUpdated(x, y, theta);
        }, Qt::QueuedConnection);

    } catch (const std::exception& e) {
        qCritical() << "处理目标点回调时出错:" << e.what();
    }
#else
    Q_UNUSED(msg);
    qWarning() << "ROS2未启用，无法处理目标点回调";
#endif
}

bool RosMapBridge::getRobotPoseFromTF(float& x, float& y, float& theta)
{
#ifdef HAVE_ROS2
    if (!ros_initialized_.load() || !tf_buffer_) {
        return false;
    }

    try {
        // 检查时间间隔，避免频繁查询
        qint64 current_time = QDateTime::currentMSecsSinceEpoch();
        if (current_time - last_tf_query_time_ < (1000 / tf_update_rate_)) {
            return false;
        }

        last_tf_query_time_ = current_time;

        // 获取当前时间
        auto now = ros_node_->now();

        // 查询TF变换
        geometry_msgs::msg::TransformStamped transform;
        transform = tf_buffer_->lookupTransform(
            map_frame_.toStdString(),
            robot_frame_.toStdString(),
            now
        );

        // 提取位置
        x = transform.transform.translation.x;
        y = transform.transform.translation.y;

        // 从四元数计算偏航角
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        theta = static_cast<float>(yaw);

        // 更新缓存
        QMutexLocker locker(&data_mutex_);
        current_robot_pose_.x = x;
        current_robot_pose_.y = y;
        current_robot_pose_.theta = theta;
        current_robot_pose_.timestamp = current_time;

        robot_pose_available_.store(true);

        return true;

    } catch (const tf2::TransformException& ex) {
        qWarning() << "TF查询失败:" << ex.what();
        return false;
    } catch (const std::exception& e) {
        qCritical() << "TF查询时发生异常:" << e.what();
        return false;
    }
#else
    Q_UNUSED(x); Q_UNUSED(y); Q_UNUSED(theta);
    qWarning() << "ROS2未启用，无法查询TF";
    return false;
#endif
}

void RosMapBridge::sendGoalPose(float x, float y, float theta)
{
    Q_UNUSED(x); Q_UNUSED(y); Q_UNUSED(theta);
    qDebug() << "发送目标点功能暂未实现";
    emit statusChanged("发送目标点功能暂未实现");
}
void RosMapBridge::requestMap()
{
#ifdef HAVE_ROS2
    if (!ros_initialized_.load() || !ros_node_) {
        qWarning() << "ROS2未就绪，无法请求地图";
        return;
    }

    qDebug() << "主动请求地图...";

    // 重新创建订阅来触发transient_local数据
    if (map_subscription_) {
        map_subscription_.reset();
    }

    rclcpp::QoS map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .transient_local()
        .reliable();

    auto map_callback = [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->mapCallback(msg);
    };

    map_subscription_ = ros_node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_.toStdString(),
        map_qos,
        map_callback
    );

    qDebug() << "地图订阅已重新创建，等待地图数据...";
#endif
}
