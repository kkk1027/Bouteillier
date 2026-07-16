// manual_controller.hpp
#ifndef MANUAL_CONTROLLER_HPP
#define MANUAL_CONTROLLER_HPP

#include <QObject>
#include <QProcess>
#include <QTimer>
#include <atomic>

class ManualController : public QObject
{
    Q_OBJECT

public:
    explicit ManualController(QObject *parent = nullptr);
    ~ManualController();

    enum ControlCommand {
        CMD_STOP = 0,
        CMD_FORWARD,
        CMD_BACKWARD,
        CMD_LEFT,
        CMD_RIGHT,
        CMD_FORWARD_LEFT,
        CMD_FORWARD_RIGHT,
        CMD_BACKWARD_LEFT,
        CMD_BACKWARD_RIGHT
    };

    // 初始化（启动Python桥接节点）
    bool initROS2();

    // 关闭ROS2节点
    void shutdownROS2();

    // 发送控制命令（毫秒级响应）
    void sendCommand(ControlCommand cmd);

    // 停止机器人
    void stop();

    // 速度控制
    void increaseSpeed();
    void decreaseSpeed();
    void resetSpeed();

    // 获取当前速度
    double getLinearSpeed() const { return linear_speed_; }
    double getAngularSpeed() const { return angular_speed_; }

    // 检查状态
    bool isInitialized() const { return bridge_initialized_; }

signals:
    void speedChanged(double linear, double angular);

private:
    // 启动Python桥接脚本
    bool startPythonBridge();

    // 发送速度命令到Python桥接
    void sendVelocityCommand(double linear, double angular);

    // 更新当前速度
    void updateCurrentVelocity(double linear, double angular);

    // 预计算的命令系数
    struct CommandCoeff {
        double linear;
        double angular;
    };

    CommandCoeff command_coeffs_[9];

    // 速度参数
    double linear_speed_;
    double angular_speed_;

    // 状态标志
    std::atomic<bool> bridge_initialized_{false};
    std::atomic<bool> bridge_running_{false};

    // Python桥接进程
    QProcess* bridge_process_;

    // 当前速度
    double current_linear_;
    double current_angular_;

    // 保持活跃定时器
    QTimer* keepalive_timer_;

    // Python脚本路径
    QString python_script_path_;

    // 速度范围
    const double LINEAR_INCREMENT = 0.05;
    const double ANGULAR_INCREMENT = 0.1;
    const double LINEAR_MAX = 0.5;
    const double ANGULAR_MAX = 1.0;
    const double LINEAR_MIN = 0.05;
    const double ANGULAR_MIN = 0.1;
};

#endif // MANUAL_CONTROLLER_HPP
