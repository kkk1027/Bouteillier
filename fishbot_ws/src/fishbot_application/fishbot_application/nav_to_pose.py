from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler
import time


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
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(
            f'预计: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s 后到达')
        # 每秒刷新 nav_status=True，维持 ESP32 心跳防止丢包触发闲时降噪
        nav_status_pub.publish(Bool(data=True))
        # 超时自动取消
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            navigator.cancelTask()
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