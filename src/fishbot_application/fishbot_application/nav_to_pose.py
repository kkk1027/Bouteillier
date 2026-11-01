from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler
import time


# ============================================================
# 僵尸防护（2026-11-01 实机事故）
# ------------------------------------------------------------
# 现场：nav2 栈被关掉之后，本节点还在跑，日志里每秒刷
#       预计: 9.098794967 s 后到达
#      预计: 9.098794967 s 后到达      ← 同一行重复了几十次
# 原因：nav2 没了 → action goal 永远没有 result →
#       navigator.isTaskComplete() 永远 False → while 死循环。
#       又因为 stdout/stderr 被 DEVNULL、父进程（web_controller）
#       重启后也失去了对它的管控 → 变成刷新不掉的僵尸进程，
#       一直占着 /navigate_to_pose 的 action client。
#
# 原来唯一的兜底是 `feedback.navigation_time > 600s`，
# 但 nav2 掉线时 feedback 根本不更新，那个条件永远不成立。
#
# 现在加两道保险：
#   1. 反馈「没变化」超时：连续 STALL_TIMEOUT 秒拿到同一份
#      estimated_time_remaining → 判定失去连接，主动退出；
#   2. 墙钟总超时：从启动算起 MAX_WALL_SEC 秒还没结果 → 退出。
# ============================================================
STALL_TIMEOUT_SEC = 20.0      # 反馈连续不变化多久就认为失联
MAX_WALL_SEC = 900.0          # 单次导航墙钟硬上限


def main():
    rclpy.init()
    navigator = BasicNavigator()
    # 告知 ESP32 导航节点已启动（>3s 无 cmd_vel 也不会触发闲时降噪）
    nav_status_pub = navigator.create_publisher(Bool, '/nav_node_status', 10)
    nav_status_pub.publish(Bool(data=True))
    # 等待 Nav2 全部生命周期节点就绪
    navigator.waitUntilNav2Active()
    # 设置目标点坐标
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 0.0
    # 使用正确的四元数：朝向0弧度
    q = quaternion_from_euler(0, 0, 0)
    goal_pose.pose.orientation.x = q[0]
    goal_pose.pose.orientation.y = q[1]
    goal_pose.pose.orientation.z = q[2]
    goal_pose.pose.orientation.w = q[3]
    # 发送目标接收反馈结果
    navigator.goToPose(goal_pose)

    wall_start = time.time()
    last_eta = None
    last_eta_change = time.time()

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback is not None:
            eta = Duration.from_msg(
                feedback.estimated_time_remaining).nanoseconds / 1e9
            if last_eta is None or abs(eta - last_eta) > 1e-3:
                last_eta = eta
                last_eta_change = time.time()
            navigator.get_logger().info(f'预计: {eta} s 后到达')
            # 超时自动取消
            if Duration.from_msg(feedback.navigation_time) > Duration(
                    seconds=600.0):
                navigator.cancelTask()
        else:
            navigator.get_logger().warn('尚未收到导航反馈')

        now = time.time()
        # ---- 保险 1：反馈长时间不变化 → nav2 大概率已掉线 ----
        if now - last_eta_change > STALL_TIMEOUT_SEC:
            navigator.get_logger().error(
                f'⚠️ 反馈已 {STALL_TIMEOUT_SEC:.0f}s 无变化，'
                f'判定与 Nav2 失联，主动退出（避免变成僵尸进程）')
            break
        # ---- 保险 2：墙钟总超时 ----
        if now - wall_start > MAX_WALL_SEC:
            navigator.get_logger().error(
                f'⚠️ 导航已运行 {MAX_WALL_SEC:.0f}s 仍无结果，主动退出')
            navigator.cancelTask()
            break

        # 每秒刷新 nav_status=True，维持 ESP32 心跳防止丢包触发闲时降噪
        nav_status_pub.publish(Bool(data=True))
        time.sleep(1.0)
    # 最终结果判断
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('导航结果：成功')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('导航结果：被取消')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('导航结果：失败')
    else:
        navigator.get_logger().error('导航结果：返回状态无效')

    # 通知 ESP32 导航结束，允许闲时降噪
    nav_status_pub.publish(Bool(data=False))
    navigator.get_logger().info('已通知 ESP32 导航结束，允许闲时降噪')
    # 多次 publish + spin 确保消息一定从 DDS 发出：
    # publish() 只是放入 DDS Writer 队列，不保证已物理发送。
    # spin_once 让 DDS 底层线程有时间把队列中消息推上网络。
    # 单次 spin_once 不够可靠（DDS 线程调度竞争），轮询多次覆盖。
    for _ in range(5):
        nav_status_pub.publish(Bool(data=False))
        rclpy.spin_once(navigator, timeout_sec=0.1)
        time.sleep(0.05)
    # 正确销毁节点和关闭 rclpy
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()