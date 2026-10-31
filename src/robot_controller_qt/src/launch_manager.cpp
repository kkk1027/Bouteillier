#include "launch_manager.hpp"
#include <iostream>
#include <QDebug>
#include <QCoreApplication>
#include <QFile>
#include <QDir>
#include <QRegularExpression>
#include <functional>

#ifdef Q_OS_LINUX
#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <dirent.h>
#endif

LaunchManager::LaunchManager(QObject *parent)
    : QObject(parent)
    , robot_process_(nullptr)
    , is_connected_(false)
    , launch_path_("/home/ke/chapt9_update/fishbot_ws/src/fishbot_bringup/launch/bringup.launch.py")
    , map_process_(nullptr)
    , navigation_process_(nullptr)
    , map_loaded_(false)
    , navigation_running_(false)
    , nav_queue_running_(false)
{
    // 设置清理定时器
    cleanup_timer_ = new QTimer(this);
    cleanup_timer_->setInterval(2000); // 2秒检查一次
    connect(cleanup_timer_, &QTimer::timeout, this, &LaunchManager::checkAndCleanup);
}

LaunchManager::~LaunchManager()
{
    std::cout << "清理LaunchManager..." << std::endl;
    forceStopAllProcesses();
}

// === 环境设置 ===
void LaunchManager::setupProcessEnvironment(QProcess* process)
{
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();

    // 设置ROS2环境变量
    env.insert("ROS_DOMAIN_ID", "0");
    env.insert("ROS_DISTRO", "humble");

    // 设置AMENT前缀路径
    QString amentPath = env.value("AMENT_PREFIX_PATH");
    if (amentPath.isEmpty()) {
        amentPath = "/opt/ros/humble";
    } else if (!amentPath.contains("/opt/ros/humble")) {
        amentPath = "/opt/ros/humble:" + amentPath;
    }
    env.insert("AMENT_PREFIX_PATH", amentPath);

    // 设置PYTHONPATH
    QString pythonPath = env.value("PYTHONPATH");
    QString rosPythonPath = "/opt/ros/humble/lib/python3.10/site-packages";
    if (pythonPath.isEmpty()) {
        pythonPath = rosPythonPath;
    } else if (!pythonPath.contains(rosPythonPath)) {
        pythonPath = rosPythonPath + ":" + pythonPath;
    }
    env.insert("PYTHONPATH", pythonPath);

    // 设置PATH
    QString path = env.value("PATH");
    if (!path.contains("/opt/ros/humble/bin")) {
        path = "/opt/ros/humble/bin:" + path;
    }
    env.insert("PATH", path);

    // 设置工作空间环境
    QString workspacePath = "/home/ke/chapt9_update/fishbot_ws";
    if (QDir(workspacePath + "/install").exists()) {
        QString installPath = workspacePath + "/install";
        env.insert("COLCON_PREFIX_PATH", installPath);

        // 添加工作空间的setup.bash
        QString setupPath = installPath + "/setup.bash";
        if (QFile::exists(setupPath)) {
            // 通过bash获取完整环境
            QProcess envProcess;
            QString command = QString("source %1 && env").arg(setupPath);
            envProcess.start("bash", QStringList() << "-c" << command);
            envProcess.waitForFinished(5000);

            QByteArray envOutput = envProcess.readAllStandardOutput();
            QStringList envLines = QString::fromLocal8Bit(envOutput).split("\n");

            for (const QString &line : envLines) {
                int index = line.indexOf("=");
                if (index > 0) {
                    QString key = line.left(index);
                    QString value = line.mid(index + 1);

                    // 只设置关键的ROS环境变量
                    if (key.contains("AMENT") || key.contains("ROS") ||
                        key.contains("PYTHON") || key.contains("LD_LIBRARY")) {
                        env.insert(key, value);
                    }
                }
            }
        }
    }

    process->setProcessEnvironment(env);
}

// === 新增导航环境设置 ===
void LaunchManager::setupNavigationEnvironment(QProcess* process)
{
    setupProcessEnvironment(process); // 复用基础环境设置

    // 为导航添加额外的环境配置
    QProcessEnvironment env = process->processEnvironment();

    // 确保导航相关的工作空间路径已设置
    QString workspacePath = "/home/ke/chapt9_update/fishbot_ws";
    if (QDir(workspacePath + "/install").exists()) {
        QString installPath = workspacePath + "/install";

        // 检查导航包是否存在
        QString navPackagePath = workspacePath + "/src/fishbot_navigation2";
        if (QDir(navPackagePath).exists()) {
            // 添加导航包特定环境
            env.insert("FISHBOT_NAVIGATION2_DIR", navPackagePath);
        }
    }

    process->setProcessEnvironment(env);
}

// === 启动机器人连接 ===
bool LaunchManager::startRobotConnection()
{
    if (is_connected_) {
        std::cout << "机器人已连接" << std::endl;
        return false;
    }

    // 先清理可能残留的进程
    forceStopAllProcesses();

    // 检查端口占用
    if (isPortOccupied(8888)) {
        std::cout << "端口8888被占用，清理中..." << std::endl;
        system("pkill -f 'micro_ros_agent'");
        sleep(1);
    }

    // 检查串口占用
    if (QFile::exists("/tmp/tty_laser")) {
        std::cout << "清理串口文件..." << std::endl;
        QFile::remove("/tmp/tty_laser");
    }

    std::cout << "启动机器人连接..." << std::endl;

    robot_process_ = new QProcess(this);
    child_pids_.clear();

    // 连接信号
    connect(robot_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &LaunchManager::onProcessFinished);
    connect(robot_process_, &QProcess::errorOccurred,
            this, &LaunchManager::onProcessError);
    connect(robot_process_, &QProcess::readyReadStandardOutput,
            this, &LaunchManager::onReadyReadStandardOutput);
    connect(robot_process_, &QProcess::readyReadStandardError,
            this, &LaunchManager::onReadyReadStandardError);

    // 设置环境
    setupProcessEnvironment(robot_process_);

    // 设置工作目录
    QString workspaceDir = "/home/ke/chapt9_update/fishbot_ws";
    robot_process_->setWorkingDirectory(workspaceDir);

    // 使用完整的source命令确保环境正确
    QString program = "bash";
    QStringList arguments;
    arguments << "-c"
              << QString("source /opt/ros/humble/setup.bash && "
                         "source %1/install/setup.bash && "
                         "echo '环境加载完成' && "
                         "ros2 launch %2")
                         .arg(workspaceDir)
                         .arg(launch_path_);

    std::cout << "执行命令: " << program.toStdString() << " " << arguments.join(" ").toStdString() << std::endl;

    robot_process_->start(program, arguments);

    if (robot_process_->waitForStarted(5000)) {
        std::cout << "机器人连接进程已启动" << std::endl;

        // 记录主进程PID
        child_pids_.append(robot_process_->processId());

        // 启动清理检查定时器
        cleanup_timer_->start();

        is_connected_ = true;
        emit connectionStarted();
        return true;
    } else {
        std::cerr << "启动失败: " << robot_process_->errorString().toStdString() << std::endl;
        emit connectionError("启动超时: " + robot_process_->errorString());
        delete robot_process_;
        robot_process_ = nullptr;
        return false;
    }
}

bool LaunchManager::stopRobotConnection()
{
    if (!is_connected_ || !robot_process_) {
        return false;
    }

    std::cout << "停止机器人连接..." << std::endl;

    // 停止定时器
    cleanup_timer_->stop();

    // 1. 先发送SIGINT (Ctrl+C)
    robot_process_->terminate();

    // 2. 等待优雅退出
    if (!robot_process_->waitForFinished(3000)) {
        std::cout << "进程未在3秒内退出，发送SIGKILL..." << std::endl;
        robot_process_->kill();
        robot_process_->waitForFinished(2000);
    }

    // 3. 清理子进程（只清理我们自己启动的）
    findAndKillChildProcesses();

    // ========== 修改：注释掉危险的强制清理 ==========
    // 原来的 forceStopAllProcesses() 会误杀系统进程，现在只做精确清理
    std::cout << "清理ROS2相关进程..." << std::endl;

    // 只清理确定是我们启动的进程，使用精确匹配
    system("pkill -9 -f 'micro_ros_agent' 2>/dev/null");
    system("pkill -9 -f 'ydlidar' 2>/dev/null");
    system("pkill -9 -f 'robot_control_bridge.py' 2>/dev/null");

    // 移除危险的 pkill -f 'python3' 和 pkill -f 'ros2'
    // 这些会误杀系统进程，导致黑屏

    // 清理串口文件
    if (QFile::exists("/tmp/tty_laser")) {
        QFile::remove("/tmp/tty_laser");
    }
    // ============================================

    std::cout << "机器人连接已停止" << std::endl;

    is_connected_ = false;
    robot_process_ = nullptr;
    child_pids_.clear();

    emit connectionStopped();
    return true;
}
// === 新增通用启动方法 ===
bool LaunchManager::startLaunchFileWithArgs(const QString& launch_file, const QString& args,
                                           QProcess*& process, QList<qint64>& pid_list,
                                           const QString& name)
{
    if (process && process->state() == QProcess::Running) {
        std::cout << name.toStdString() << " 已在运行" << std::endl;
        return false;
    }

    // 清理可能的残留进程
    if (process) {
        delete process;
        process = nullptr;
    }

    process = new QProcess(this);
    pid_list.clear();

    // 连接信号
    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &LaunchManager::onProcessFinished);
    connect(process, &QProcess::errorOccurred,
            this, &LaunchManager::onProcessError);
    connect(process, &QProcess::readyReadStandardOutput,
            this, &LaunchManager::onReadyReadStandardOutput);
    connect(process, &QProcess::readyReadStandardError,
            this, &LaunchManager::onReadyReadStandardError);

    // 设置环境
    setupNavigationEnvironment(process);

    // 设置工作目录
    QString workspaceDir = "/home/ke/chapt9_update/fishbot_ws";
    process->setWorkingDirectory(workspaceDir);

    // 构建命令，包含参数
    QString program = "bash";
    QStringList arguments;

    // 如果提供了参数，则在 ros2 launch 命令中添加
    QString ros2_command;
    if (args.isEmpty()) {
        ros2_command = QString("ros2 launch %1").arg(launch_file);
    } else {
        ros2_command = QString("ros2 launch %1 %2").arg(launch_file).arg(args);
    }

    arguments << "-c"
              << QString("source /opt/ros/humble/setup.bash && "
                         "source %1/install/setup.bash && "
                         "echo '启动%2...' && "
                         "%3")
                         .arg(workspaceDir)
                         .arg(name)
                         .arg(ros2_command);

    std::cout << "执行" << name.toStdString() << "命令: "
              << program.toStdString() << " " << arguments.join(" ").toStdString() << std::endl;

    process->start(program, arguments);

    if (process->waitForStarted(5000)) {
        std::cout << name.toStdString() << " 进程已启动" << std::endl;

        // 记录主进程PID
        pid_list.append(process->processId());

        return true;
    } else {
        std::cerr << "启动" << name.toStdString() << "失败: "
                  << process->errorString().toStdString() << std::endl;
        delete process;
        process = nullptr;
        return false;
    }
}

bool LaunchManager::startLaunchFile(const QString& launch_file, QProcess*& process,
                                   QList<qint64>& pid_list, const QString& name)
{
    if (process && process->state() == QProcess::Running) {
        std::cout << name.toStdString() << " 已在运行" << std::endl;
        return false;
    }

    // 清理可能的残留进程
    if (process) {
        delete process;
        process = nullptr;
    }

    process = new QProcess(this);
    pid_list.clear();

    // 连接信号
    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &LaunchManager::onProcessFinished);
    connect(process, &QProcess::errorOccurred,
            this, &LaunchManager::onProcessError);
    connect(process, &QProcess::readyReadStandardOutput,
            this, &LaunchManager::onReadyReadStandardOutput);
    connect(process, &QProcess::readyReadStandardError,
            this, &LaunchManager::onReadyReadStandardError);

    // 设置环境
    setupNavigationEnvironment(process);

    // 设置工作目录
    QString workspaceDir = "/home/ke/chapt9_update/fishbot_ws";
    process->setWorkingDirectory(workspaceDir);

    // 构建命令
    QString program = "bash";
    QStringList arguments;
    arguments << "-c"
              << QString("source /opt/ros/humble/setup.bash && "
                         "source %1/install/setup.bash && "
                         "echo '启动%2...' && "
                         "ros2 launch %3")
                         .arg(workspaceDir)
                         .arg(name)
                         .arg(launch_file);

    std::cout << "执行" << name.toStdString() << "命令: "
              << program.toStdString() << " " << arguments.join(" ").toStdString() << std::endl;

    process->start(program, arguments);

    if (process->waitForStarted(5000)) {
        std::cout << name.toStdString() << " 进程已启动" << std::endl;

        // 记录主进程PID
        pid_list.append(process->processId());

        return true;
    } else {
        std::cerr << "启动" << name.toStdString() << "失败: "
                  << process->errorString().toStdString() << std::endl;
        delete process;
        process = nullptr;
        return false;
    }
}

// === 新增通用停止方法 ===
bool LaunchManager::stopProcess(QProcess*& process, QList<qint64>& pid_list, const QString& name)
{
    if (!process || process->state() != QProcess::Running) {
        return false;
    }

    std::cout << "停止" << name.toStdString() << "..." << std::endl;

    // 1. 先发送SIGINT (Ctrl+C)
    process->terminate();

    // 2. 等待优雅退出
    if (!process->waitForFinished(3000)) {
        std::cout << name.toStdString() << "进程未在3秒内退出，发送SIGKILL..." << std::endl;

        // 3. 强制终止
        process->kill();
        process->waitForFinished(2000);
    }

    // 4. 清理子进程
    for (qint64 pid : pid_list) {
        killProcessTree(pid);
    }

    pid_list.clear();

    delete process;
    process = nullptr;

    std::cout << name.toStdString() << " 已停止" << std::endl;
    return true;
}

// === 新增进程树终止方法 ===
void LaunchManager::killProcessTree(qint64 pid)
{
    if (pid <= 0) return;

#ifdef Q_OS_LINUX
    // 查找并杀死所有子进程
    QList<qint64> child_pids;
    findAndKillProcessTree(pid, child_pids);

    // 杀死父进程
    kill(pid, SIGKILL);
#endif
}

void LaunchManager::findAndKillProcessTree(qint64 parent_pid, QList<qint64>& pid_list)
{
#ifdef Q_OS_LINUX
    QFile file(QString("/proc/%1/task/%2/children").arg(parent_pid).arg(parent_pid));
    if (file.open(QIODevice::ReadOnly)) {
        QString data = QString::fromLocal8Bit(file.readAll()).trimmed();
        if (!data.isEmpty()) {
            for (const QString& pidStr : data.split(" ", Qt::SkipEmptyParts)) {
                qint64 childPid = pidStr.toLongLong();
                if (childPid > 0) {
                    pid_list.append(childPid);

                    // 递归查找孙子进程
                    findAndKillProcessTree(childPid, pid_list);

                    // 杀死子进程
                    kill(childPid, SIGKILL);
                }
            }
        }
        file.close();
    }
#endif
}

// === 启动地图和RVIZ ===
// launch_manager.cpp - 修改 startMapAndRVIZ 方法
bool LaunchManager::startMapAndRVIZ()
{
    if (map_loaded_) {
        std::cout << "地图已加载" << std::endl;
        return true;
    }

    QString navigation_launch_path = "/home/ke/chapt9_update/fishbot_ws/src/fishbot_navigation2/launch/navigation2.launch.py";

    if (!QFile::exists(navigation_launch_path)) {
        emit mapLoaded(false, "导航launch文件不存在: " + navigation_launch_path);
        return false;
    }

    std::cout << "启动地图和RVIZ（仅地图，不启动导航）..." << std::endl;

    // 修改：添加 use_sim_time:=False 参数
    bool success = startLaunchFileWithArgs(navigation_launch_path, "use_sim_time:=False",
                                           map_process_, map_child_pids_, "地图和RVIZ");

    if (success) {
        map_loaded_ = true;

        // 延迟发射成功信号，确保进程稳定
        QTimer::singleShot(3000, [this]() {
            emit mapLoaded(true, "地图和RVIZ加载成功 - 等待启动导航");
            std::cout << "地图加载完成，等待用户手动启动导航" << std::endl;
        });
    } else {
        emit mapLoaded(false, "启动地图和RVIZ失败");
    }

    return success;
}
// === 停止地图和RVIZ ===
bool LaunchManager::stopMapAndRVIZ()
{
    if (!map_loaded_) {
        return true;
    }

    bool success = stopProcess(map_process_, map_child_pids_, "地图和RVIZ");

    if (success) {
        map_loaded_ = false;
        emit mapStopped();

        // 如果导航正在运行，也停止它
        if (navigation_running_) {
            stopAutonomousNavigation();
        }
    }

    return success;
}

// === 启动自主导航 ===
bool LaunchManager::startAutonomousNavigation()
{
    if (!map_loaded_) {
        emit navigationStarted(false, "请先加载地图");
        return false;
    }

    if (navigation_running_) {
        std::cout << "导航已在运行" << std::endl;
        return true;
    }

    QString autopatrol_launch_path = "/home/ke/chapt9_update/fishbot_ws/src/autopatrol_robot/launch/autopatrol.launch.py";

    // 检查文件是否存在
    if (!QFile::exists(autopatrol_launch_path)) {
        // 尝试其他可能的位置
        autopatrol_launch_path = "/home/ke/chapt9_update/fishbot_ws/install/autopatrol_robot/share/autopatrol_robot/launch/autopatrol.launch.py";

        if (!QFile::exists(autopatrol_launch_path)) {
            emit navigationStarted(false, "自主导航launch文件不存在");
            std::cerr << "找不到文件: " << autopatrol_launch_path.toStdString() << std::endl;

            // 列出可能的路径
            std::cerr << "尝试查找可能的路径..." << std::endl;
            system("find /home/ke/chapt9_update/fishbot_ws -name \"autopatrol.launch*\" 2>/dev/null");

            return false;
        }
    }

    std::cout << "启动自主导航，使用文件: " << autopatrol_launch_path.toStdString() << std::endl;

    bool success = startLaunchFile(autopatrol_launch_path, navigation_process_, nav_child_pids_, "自主导航");

    if (success) {
        navigation_running_ = true;

        // 延迟发射成功信号
        QTimer::singleShot(2000, [this]() {
            emit navigationStarted(true, "自主导航启动成功 - 开始巡航");
        });
    } else {
        emit navigationStarted(false, "启动自主导航失败");
    }

    return success;
}
// === 停止自主导航 ===
bool LaunchManager::stopAutonomousNavigation()
{
    if (!navigation_running_) {
        return true;
    }

    bool success = stopProcess(navigation_process_, nav_child_pids_, "自主导航");

    if (success) {
        navigation_running_ = false;
        emit navigationStopped();
    }

    return success;
}

// === 强制终止所有相关进程 ===
bool LaunchManager::forceStopAllProcesses()
{
    std::cout << "=== 强制清理所有进程 ===" << std::endl;

    // 停止我们自己启动的进程
    if (robot_process_ && robot_process_->state() == QProcess::Running) {
        robot_process_->kill();
        robot_process_->waitForFinished(500);
    }

    if (map_process_ && map_process_->state() == QProcess::Running) {
        map_process_->kill();
        map_process_->waitForFinished(500);
    }

    if (navigation_process_ && navigation_process_->state() == QProcess::Running) {
        navigation_process_->kill();
        navigation_process_->waitForFinished(500);
    }

    // ========== 修改：移除危险的 pkill，只清理已知的特定进程 ==========
    std::cout << "清理特定ROS2进程..." << std::endl;

    // 只清理确定是我们启动的进程（精确匹配进程名或路径）
    system("pkill -9 -f 'micro_ros_agent' 2>/dev/null");
    system("pkill -9 -f 'ydlidar' 2>/dev/null");
    system("pkill -9 -f 'robot_control_bridge.py' 2>/dev/null");
    system("pkill -9 -f 'fishbot_bringup' 2>/dev/null");
    system("pkill -9 -f 'navigation2.launch' 2>/dev/null");
    system("pkill -9 -f 'autopatrol.launch' 2>/dev/null");

    // 移除以下危险的命令（会误杀系统进程）：
    // system("pkill -9 -f 'ros2' 2>/dev/null");      // 危险！
    // system("pkill -9 -f 'python3' 2>/dev/null");   // 危险！
    // system("pkill -9 -f 'ros_' 2>/dev/null");      // 危险！
    // system("pkill -9 -f 'rviz2' 2>/dev/null");     // 危险！
    // system("pkill -9 -f 'nav2' 2>/dev/null");      // 危险！

    // 清理端口占用
    system("fuser -k 8888/tcp 2>/dev/null");

    // 清理串口文件
    if (QFile::exists("/tmp/tty_laser")) {
        QFile::remove("/tmp/tty_laser");
    }

    // 等待进程完全终止
    QProcess::execute("sleep", QStringList() << "0.5");
    // ===============================================

    std::cout << "进程清理完成" << std::endl;

    // 重置状态
    is_connected_ = false;
    map_loaded_ = false;
    navigation_running_ = false;
    robot_process_ = nullptr;
    map_process_ = nullptr;
    navigation_process_ = nullptr;

    return true;
}
// === 检查连接状态 ===
bool LaunchManager::isConnected() const
{
    return is_connected_;
}

// === 检查端口占用 ===
bool LaunchManager::isPortOccupied(int port) const
{
    QString command = QString("netstat -tlnp 2>/dev/null | grep ':%1 '").arg(port);
    QProcess process;
    process.start("bash", QStringList() << "-c" << command);
    process.waitForFinished(1000);

    QString output = process.readAllStandardOutput();
    return !output.trimmed().isEmpty();
}

// === 清理子进程 ===
void LaunchManager::findAndKillChildProcesses()
{
#ifdef Q_OS_LINUX
    qint64 parentPid = robot_process_ ? robot_process_->processId() : 0;
    if (parentPid <= 0) return;

    std::function<void(qint64, QList<qint64>&)> findChildren;
    findChildren = [&findChildren](qint64 pid, QList<qint64>& pids) {
        QFile file(QString("/proc/%1/task/%2/children").arg(pid).arg(pid));
        if (file.open(QIODevice::ReadOnly)) {
            QString data = QString::fromLocal8Bit(file.readAll()).trimmed();
            if (!data.isEmpty()) {
                for (const QString& pidStr : data.split(" ", Qt::SkipEmptyParts)) {
                    qint64 childPid = pidStr.toLongLong();
                    if (childPid > 0) {
                        pids.append(childPid);
                        findChildren(childPid, pids);
                    }
                }
            }
            file.close();
        }
    };

    QList<qint64> allPids;
    findChildren(parentPid, allPids);

    // ========== 修改：先尝试 SIGTERM，再 SIGKILL ==========
    for (qint64 pid : allPids) {
        if (pid > 0) {
            // 先温和地请求终止
            kill(pid, SIGTERM);
        }
    }

    // 等待 1 秒让进程优雅退出
    sleep(1);

    // 对还在运行的进程强制终止
    for (qint64 pid : allPids) {
        if (pid > 0 && kill(pid, 0) == 0) {  // 进程还存在
            kill(pid, SIGKILL);
        }
    }
    // ===================================================
#endif
}

// === 获取进程树信息 ===
QString LaunchManager::getProcessTreeInfo() const
{
    QString info;
#ifdef Q_OS_LINUX
    qint64 parentPid = robot_process_ ? robot_process_->processId() : 0;
    if (parentPid > 0) {
        QProcess process;
        process.start("pstree", QStringList() << "-p" << QString::number(parentPid));
        process.waitForFinished(1000);
        info = process.readAllStandardOutput();
    }
#endif
    return info;
}

// === 进程结束处理 ===
void LaunchManager::onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    QProcess* process = qobject_cast<QProcess*>(sender());

    if (process == robot_process_) {
        std::cout << "机器人连接进程结束，退出码: " << exitCode << std::endl;
        is_connected_ = false;
        emit connectionStopped();
    }
    else if (process == map_process_) {
        std::cout << "地图和RVIZ进程结束，退出码: " << exitCode << std::endl;
        map_loaded_ = false;
        emit mapStopped();

        // 地图停止时也停止导航
        if (navigation_running_) {
            stopAutonomousNavigation();
        }
    }
    else if (process == navigation_process_) {
        std::cout << "自主导航进程结束，退出码: " << exitCode << std::endl;
        navigation_running_ = false;
        emit navigationStopped();
    }

    // 清理定时器
    cleanup_timer_->stop();
}

// === 进程错误处理 ===
void LaunchManager::onProcessError(QProcess::ProcessError error)
{
    QProcess* process = qobject_cast<QProcess*>(sender());
    QString processName = "未知进程";

    if (process == robot_process_) processName = "机器人连接";
    else if (process == map_process_) processName = "地图和RVIZ";
    else if (process == navigation_process_) processName = "自主导航";

    QString errorMsg = processName + "错误: ";
    switch (error) {
        case QProcess::FailedToStart: errorMsg += "进程启动失败"; break;
        case QProcess::Crashed: errorMsg += "进程崩溃"; break;
        case QProcess::Timedout: errorMsg += "操作超时"; break;
        case QProcess::WriteError: errorMsg += "写入错误"; break;
        case QProcess::ReadError: errorMsg += "读取错误"; break;
        default: errorMsg += "未知错误"; break;
    }

    std::cerr << errorMsg.toStdString() << std::endl;

    // 根据进程类型进行相应处理
    if (process == robot_process_) {
        emit connectionError(errorMsg);
    }
    else if (process == map_process_) {
        map_loaded_ = false;
        emit mapLoaded(false, errorMsg);
    }
    else if (process == navigation_process_) {
        navigation_running_ = false;
        emit navigationStarted(false, errorMsg);
    }
}

// === 标准输出处理 ===
void LaunchManager::onReadyReadStandardOutput()
{
    QProcess* process = qobject_cast<QProcess*>(sender());
    if (!process) return;

    QByteArray output = process->readAllStandardOutput();
    QString outputStr = QString::fromLocal8Bit(output);

    // 检测导航特定信息
    if (process == navigation_process_) {
        if (outputStr.contains("获取到目标点") || outputStr.contains("导航结果")) {
            std::cout << "[导航信息] " << outputStr.toStdString();
        }
    }

    // 发射信号
    emit processOutput(outputStr);
}
// === 标准错误处理 ===
void LaunchManager::onReadyReadStandardError()
{
    QProcess* process = qobject_cast<QProcess*>(sender());
    if (!process) return;

    QByteArray error = process->readAllStandardError();
    QString errorStr = QString::fromLocal8Bit(error);

    std::cerr << errorStr.toStdString();
    emit processOutput("[ERROR] " + errorStr);
}

// === 定时清理检查 ===
void LaunchManager::checkAndCleanup()
{
    // 定期检查进程状态
    if (robot_process_ && robot_process_->state() == QProcess::NotRunning) {
        forceStopAllProcesses();
        emit connectionStopped();
    }

    // 检查并清理僵尸进程（静默执行）
    system("ps aux | grep -E '(micro_ros_agent|ydlidar|ros_serial2wifi)' | "
           "grep -v grep | awk '{print $2}' | xargs -r ps -o pid= --ppid 2>/dev/null 1>/dev/null");
}

bool LaunchManager::startNavigationWithQueue(const QStringList& commands)
{
    if (commands.isEmpty()) {
        std::cout << "导航命令队列为空" << std::endl;
        emit navigationStarted(false, "导航命令队列为空");
        return false;
    }

    if (!map_loaded_) {
        emit navigationStarted(false, "请先加载地图");
        return false;
    }

    if (nav_queue_running_) {
        std::cout << "导航队列已在运行" << std::endl;
        return true;
    }

    // 清空并设置新队列
    nav_command_queue_ = commands;
    nav_queue_running_ = true;

    // 开始执行第一个命令
    executeNextNavigationCommand();

    return true;
}

bool LaunchManager::isNavigationQueueRunning() const
{
    return nav_queue_running_;
}

void LaunchManager::executeNextNavigationCommand()
{
    if (nav_command_queue_.isEmpty()) {
        // 队列为空，导航完成
        nav_queue_running_ = false;
        std::cout << "所有航点导航完成" << std::endl;
        emit navigationStarted(true, "所有航点导航完成");
        return;
    }

    // 取出第一个命令
    QString command = nav_command_queue_.takeFirst();

    // 停止当前运行的导航进程（如果有）
    if (navigation_process_ && navigation_process_->state() == QProcess::Running) {
        stopProcess(navigation_process_, nav_child_pids_, "自主导航");
    }

    // 解析命令：格式为 "ros2 launch autopatrol_robot autopatrol.launch.py target_indices:=\"X\""
    // 直接使用命令字符串启动
    if (navigation_process_) {
        delete navigation_process_;
        navigation_process_ = nullptr;
    }

    navigation_process_ = new QProcess(this);
    nav_child_pids_.clear();

    // 连接信号
    connect(navigation_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &LaunchManager::onNavigationQueueFinished);
    connect(navigation_process_, &QProcess::errorOccurred,
            this, &LaunchManager::onProcessError);
    connect(navigation_process_, &QProcess::readyReadStandardOutput,
            this, &LaunchManager::onReadyReadStandardOutput);
    connect(navigation_process_, &QProcess::readyReadStandardError,
            this, &LaunchManager::onReadyReadStandardError);

    // 设置环境
    setupNavigationEnvironment(navigation_process_);

    // 设置工作目录
    QString workspaceDir = "/home/ke/chapt9_update/fishbot_ws";
    navigation_process_->setWorkingDirectory(workspaceDir);

    // 构建完整命令：source 环境后执行用户命令
    QString program = "bash";
    QStringList arguments;
    arguments << "-c"
              << QString("source /opt/ros/humble/setup.bash && "
                         "source %1/install/setup.bash && "
                         "%2")
                         .arg(workspaceDir)
                         .arg(command);

    std::cout << "执行导航命令: " << program.toStdString() << " " << arguments.join(" ").toStdString() << std::endl;

    navigation_process_->start(program, arguments);

    if (navigation_process_->waitForStarted(5000)) {
        std::cout << "导航命令进程已启动" << std::endl;
        nav_child_pids_.append(navigation_process_->processId());

        // 发射导航开始信号（每个航点开始时）
        emit navigationStarted(true, QString("正在执行: %1").arg(command));
        navigation_running_ = true;
    } else {
        std::cerr << "启动导航命令失败: " << navigation_process_->errorString().toStdString() << std::endl;
        // 失败时继续执行下一个
        executeNextNavigationCommand();
    }
}

// 新增：导航队列进程完成时的处理
void LaunchManager::onNavigationQueueFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    Q_UNUSED(exitCode);
    Q_UNUSED(exitStatus);

    std::cout << "当前导航命令执行完成，退出码: " << exitCode << std::endl;

    // 清理当前进程
    if (navigation_process_) {
        navigation_process_->deleteLater();
        navigation_process_ = nullptr;
    }
    nav_child_pids_.clear();

    // 执行下一个命令
    executeNextNavigationCommand();
}
