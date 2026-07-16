#ifndef LAUNCH_MANAGER_HPP
#define LAUNCH_MANAGER_HPP

#include <QObject>
#include <QProcess>
#include <QString>
#include <QTimer>
#include <memory>

class LaunchManager : public QObject
{
    Q_OBJECT

public:
    explicit LaunchManager(QObject *parent = nullptr);
    ~LaunchManager();

    // 启动机器人连接
    bool startRobotConnection();

    // 停止机器人连接
    bool stopRobotConnection();

    // 强制终止所有相关进程
    bool forceStopAllProcesses();

    // 检查连接状态
    bool isConnected() const;

    // 检查端口占用
    bool isPortOccupied(int port = 8888) const;

    // === 新增导航相关方法 ===
    // 启动地图加载和RVIZ
    bool startMapAndRVIZ();

    // 停止地图加载和RVIZ
    bool stopMapAndRVIZ();

    // 启动自主导航
    bool startAutonomousNavigation();

    // 停止自主导航
    bool stopAutonomousNavigation();

    // 检查地图是否已加载
    bool isMapLoaded() const { return map_loaded_; }

    // 检查导航是否正在运行
    bool isNavigationRunning() const { return navigation_running_; }

    bool startNavigationWithQueue(const QStringList& commands);
    bool isNavigationQueueRunning() const;
signals:
    void connectionStarted();
    void connectionStopped();
    void connectionError(const QString& error);
    void processOutput(const QString& output);

    // === 新增导航相关信号 ===
    void mapLoaded(bool success, const QString& message);
    void mapStopped();
    void navigationStarted(bool success, const QString& message);
    void navigationStopped();

private slots:
    void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void onProcessError(QProcess::ProcessError error);
    void onReadyReadStandardOutput();
    void onReadyReadStandardError();
    void checkAndCleanup();
    void onNavigationQueueFinished(int exitCode, QProcess::ExitStatus exitStatus);
private:
    // 主机器人连接进程
    QProcess* robot_process_;
    bool is_connected_;
    QString launch_path_;

    // === 新增导航相关进程 ===
    QProcess* map_process_;          // 地图和RVIZ进程
    QProcess* navigation_process_;   // 自主导航进程

    // === 新增状态标志 ===
    bool map_loaded_;
    bool navigation_running_;

    // 清理定时器
    QTimer* cleanup_timer_;

    // 子进程PID列表（用于基础连接）
    QList<qint64> child_pids_;

    // 新增导航相关子进程PID列表
    QList<qint64> map_child_pids_;
    QList<qint64> nav_child_pids_;

    void setupProcessEnvironment(QProcess* process);
    void findAndKillChildProcesses();
    QString getProcessTreeInfo() const;

    // === 新增导航相关私有方法 ===
    void setupNavigationEnvironment(QProcess* process);
    bool startLaunchFile(const QString& launch_file, QProcess*& process,
                                       QList<qint64>& pid_list, const QString& name);
    bool startLaunchFileWithArgs(const QString& launch_file, const QString& args,
                                               QProcess*& process, QList<qint64>& pid_list,
                                               const QString& name);
    bool stopProcess(QProcess*& process, QList<qint64>& pid_list, const QString& name);
    void killProcessTree(qint64 pid);
    void findAndKillProcessTree(qint64 parent_pid, QList<qint64>& pid_list);

    QStringList nav_command_queue_;
    bool nav_queue_running_;
    void executeNextNavigationCommand();
};

#endif // LAUNCH_MANAGER_HPP
