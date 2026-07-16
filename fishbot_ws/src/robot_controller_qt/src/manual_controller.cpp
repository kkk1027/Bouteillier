// manual_controller.cpp - 简化版本（推荐使用）
#include "manual_controller.hpp"
#include <QDir>
#include <QFile>
#include <QThread>
#include <cmath>

ManualController::ManualController(QObject *parent)
    : QObject(parent)
    , linear_speed_(0.1)
    , angular_speed_(0.25)
    , bridge_process_(nullptr)
    , current_linear_(0.0)
    , current_angular_(0.0)
{
    // 初始化命令系数
    command_coeffs_[CMD_STOP] = {0.0, 0.0};
    command_coeffs_[CMD_FORWARD] = {1.0, 0.0};
    command_coeffs_[CMD_BACKWARD] = {-1.0, 0.0};
    command_coeffs_[CMD_LEFT] = {0.0, 1.0};
    command_coeffs_[CMD_RIGHT] = {0.0, -1.0};
    command_coeffs_[CMD_FORWARD_LEFT] = {1.0, 1.0};
    command_coeffs_[CMD_FORWARD_RIGHT] = {1.0, -1.0};
    command_coeffs_[CMD_BACKWARD_LEFT] = {-1.0, 1.0};
    command_coeffs_[CMD_BACKWARD_RIGHT] = {-1.0, -1.0};

    // 设置Python脚本路径
    QString appDir = QDir::currentPath();
    python_script_path_ = appDir + "/robot_control_bridge.py";

    // 注意：完全移除定时器！不创建keepalive_timer_
}

ManualController::~ManualController()
{
    shutdownROS2();
}

bool ManualController::startPythonBridge()
{
    if (bridge_process_) {
        // 清理旧的进程
        delete bridge_process_;
        bridge_process_ = nullptr;
    }

    // 检查Python脚本是否存在
    if (!QFile::exists(python_script_path_)) {
        QString workspaceDir = qEnvironmentVariable("FISHBOT_WS", "/home/ke/chapt9_update/fishbot_ws");
        QString alternative_path = workspaceDir + "/src/robot_controller_qt/robot_control_bridge.py";
        if (QFile::exists(alternative_path)) {
            python_script_path_ = alternative_path;
        } else {
            return false;
        }
    }

    // 创建新的进程
    bridge_process_ = new QProcess(this);

    // 设置进程为标准输入模式
    bridge_process_->setProcessChannelMode(QProcess::ForwardedChannels);

    // 连接信号
    connect(bridge_process_, &QProcess::errorOccurred, [this](QProcess::ProcessError error) {
        bridge_running_ = false;
        bridge_initialized_ = false;
    });

    connect(bridge_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            [this](int exitCode, QProcess::ExitStatus exitStatus) {
        bridge_running_ = false;
        bridge_initialized_ = false;
    });

    // 设置环境变量 - 简化版本
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert("ROS_DOMAIN_ID", "0");
    env.insert("ROS_DISTRO", "humble");

    // 确保Python能找到ROS2
    QString pythonpath = env.value("PYTHONPATH", "");
    QString ros_python_path = "/opt/ros/humble/lib/python3.10/site-packages";
    if (!pythonpath.contains(ros_python_path)) {
        pythonpath = ros_python_path + (pythonpath.isEmpty() ? "" : ":" + pythonpath);
        env.insert("PYTHONPATH", pythonpath);
    }

    bridge_process_->setProcessEnvironment(env);

    // 启动Python脚本
    bridge_process_->start("python3", QStringList() << python_script_path_);

    // 等待进程启动
    if (bridge_process_->waitForStarted(2000)) {
        bridge_running_ = true;

        // 等待Python脚本初始化
        QThread::msleep(300);  // 等待300ms让Python脚本启动

        // 发送测试命令检查连接
        QString test_command = "0.0,0.0\n";
        bridge_process_->write(test_command.toUtf8());
        bridge_process_->waitForBytesWritten(100);

        bridge_initialized_ = true;
        return true;
    }

    delete bridge_process_;
    bridge_process_ = nullptr;
    return false;
}

bool ManualController::initROS2()
{
    if (bridge_initialized_) {
        return true;
    }

    // 快速检查Python3
    QProcess pythonCheck;
    pythonCheck.start("which", QStringList() << "python3");
    pythonCheck.waitForFinished(200);

    if (pythonCheck.exitCode() != 0) {
        return false;
    }

    // 启动Python桥接
    return startPythonBridge();
}

void ManualController::shutdownROS2()
{
    if (bridge_process_) {
        // 发送停止命令
        QString stop_command = "STOP\n";
        bridge_process_->write(stop_command.toUtf8());
        bridge_process_->waitForBytesWritten(100);

        // 等待一下
        QThread::msleep(50);

        // 终止进程
        bridge_process_->terminate();
        if (!bridge_process_->waitForFinished(1000)) {
            bridge_process_->kill();
            bridge_process_->waitForFinished(500);
        }

        delete bridge_process_;
        bridge_process_ = nullptr;
    }

    bridge_initialized_ = false;
    bridge_running_ = false;
}

void ManualController::sendVelocityCommand(double linear, double angular)
{
    if (!bridge_process_ || !bridge_running_ || !bridge_initialized_) {
        return;
    }

    // 检查进程状态
    if (bridge_process_->state() != QProcess::Running) {
        bridge_running_ = false;
        bridge_initialized_ = false;
        return;
    }

    // 格式化命令
    QString command = QString("%1,%2\n")
                     .arg(linear, 0, 'f', 3)
                     .arg(angular, 0, 'f', 3);

    QByteArray data = command.toUtf8();

    // 简单直接地发送命令
    qint64 bytesWritten = bridge_process_->write(data);

    if (bytesWritten > 0) {
        // 短暂等待确保写入
        bridge_process_->waitForBytesWritten(10);
    }
}

void ManualController::sendCommand(ControlCommand cmd)
{
    if (!bridge_initialized_) {
        return;
    }

    // 获取预计算的系数
    const CommandCoeff& coeff = command_coeffs_[cmd];

    // 计算实际速度
    double linear = coeff.linear * linear_speed_;
    double angular = coeff.angular * angular_speed_;

    // 更新当前速度
    current_linear_ = linear;
    current_angular_ = angular;

    // 立即发送命令
    sendVelocityCommand(linear, angular);
}

void ManualController::stop()
{
    if (!bridge_initialized_) {
        return;
    }

    // 发送特殊停止命令
    if (bridge_process_ && bridge_process_->state() == QProcess::Running) {
        // 发送多次停止命令确保接收
        QStringList stop_commands = {
            "STOP\n",
            "0.0,0.0\n",
            "0.000,0.000\n"
        };

        for (const QString& cmd : stop_commands) {
            bridge_process_->write(cmd.toUtf8());
            bridge_process_->waitForBytesWritten(20);
            QThread::msleep(5);
        }
    }

    // 更新内部状态
    current_linear_ = 0.0;
    current_angular_ = 0.0;
}

// 速度控制方法保持不变
void ManualController::increaseSpeed()
{
    // 保存旧速度用于计算比例
    double old_linear = linear_speed_;
    double old_angular = angular_speed_;

    // 增加速度
    linear_speed_ += LINEAR_INCREMENT;
    angular_speed_ += ANGULAR_INCREMENT;

    // 限制最大速度
    if (linear_speed_ > LINEAR_MAX) linear_speed_ = LINEAR_MAX;
    if (angular_speed_ > ANGULAR_MAX) angular_speed_ = ANGULAR_MAX;

    // 发射速度变化信号
    emit speedChanged(linear_speed_, angular_speed_);

    // 如果当前正在运动，按比例更新速度
    // 重要：只有当旧速度不为0时才能计算比例
    if ((current_linear_ != 0.0 || current_angular_ != 0.0) &&
        fabs(old_linear) > 0.001 && fabs(old_angular) > 0.001) {

        // 计算速度系数
        double linear_coeff = current_linear_ / old_linear;
        double angular_coeff = current_angular_ / old_angular;

        // 应用新速度
        double new_linear = linear_coeff * linear_speed_;
        double new_angular = angular_coeff * angular_speed_;

        // 更新并发送新速度
        updateCurrentVelocity(new_linear, new_angular);
    }
    // 如果是从停止状态加速，或者旧速度为0，保持当前运动状态不变
    // 用户需要重新按下方向按钮才会应用新速度
}
void ManualController::decreaseSpeed()
{
    // 保存旧速度用于计算比例
    double old_linear = linear_speed_;
    double old_angular = angular_speed_;

    // 减少速度
    linear_speed_ -= LINEAR_INCREMENT;
    angular_speed_ -= ANGULAR_INCREMENT;

    // 限制最小速度
    if (linear_speed_ < LINEAR_MIN) linear_speed_ = LINEAR_MIN;
    if (angular_speed_ < ANGULAR_MIN) angular_speed_ = ANGULAR_MIN;

    // 发射速度变化信号
    emit speedChanged(linear_speed_, angular_speed_);

    // 如果当前正在运动，按比例更新速度
    // 重要：只有当旧速度不为0时才能计算比例
    if ((current_linear_ != 0.0 || current_angular_ != 0.0) &&
        fabs(old_linear) > 0.001 && fabs(old_angular) > 0.001) {

        // 计算速度系数
        double linear_coeff = current_linear_ / old_linear;
        double angular_coeff = current_angular_ / old_angular;

        // 应用新速度
        double new_linear = linear_coeff * linear_speed_;
        double new_angular = angular_coeff * angular_speed_;

        // 更新并发送新速度
        updateCurrentVelocity(new_linear, new_angular);
    }
    // 如果是从停止状态减速，保持当前运动状态不变
}
void ManualController::resetSpeed() { /* 保持不变 */ }
void ManualController::updateCurrentVelocity(double linear, double angular)
{
    // 更新当前速度
    current_linear_ = linear;
    current_angular_ = angular;

    // 发送新的速度命令
    sendVelocityCommand(linear, angular);
}
