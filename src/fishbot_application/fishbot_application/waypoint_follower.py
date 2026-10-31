import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_srvs.srv import SetBool
import threading
class ServoClient:
    """舵机控制客户端（Python端）"""
    def __init__(self):
        # rclpy.init()
        self.node = Node('servo_client_python')
        
    def call_servo_action(self):
        """调用舵机动作服务（通过ESP32的SetBool服务）"""
        from rclpy.callback_groups import ReentrantCallbackGroup
        client = self.node.create_client(
            SetBool, 
            '/esp32_command',
            callback_group=ReentrantCallbackGroup()
        )
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('ESP32命令服务不可用')
            return False
        
        req = SetBool.Request()
        req.data = True  # 触发舵机动作
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        
        if future.result().success:
            self.node.get_logger().info('ESP32命令执行成功（舵机动作）')
            return True
        else:
            self.node.get_logger().error('ESP32命令执行失败')
            return False
def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    servo_client = ServoClient()
    # 创建点集
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 0.0
    goal_pose1.pose.position.y = -0.5
    goal_pose1.pose.orientation.w = 1.0
    goal_poses.append(goal_pose1)
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.0
    goal_pose2.pose.position.y = -0.5
    goal_pose2.pose.orientation.w = 1.0
    goal_poses.append(goal_pose2)
    # goal_pose3 = PoseStamped()
    # goal_pose3.header.frame_id = 'map'
    # goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose3.pose.position.x = 0.0
    # goal_pose3.pose.position.y = 0.0
    # goal_pose3.pose.orientation.w = 1.0
    # goal_poses.append(goal_pose3)
    # 调用路点导航服务
    navigator.followWaypoints(goal_poses)
    # 判断结束及获取反馈
    current_waypoint = -1
    servo_active = False
    executed_waypoints = set()
        
        # 导航循环
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        
        if feedback and hasattr(feedback, 'current_waypoint'):
            new_waypoint = feedback.current_waypoint
            
            # 记录进度
            if new_waypoint != current_waypoint:
                navigator.get_logger().info(f'当前航点: {new_waypoint + 1}/{len(goal_poses)}')
                current_waypoint = new_waypoint
            
            # 检测新到达的航点（未执行过舵机动作）
            if (new_waypoint not in executed_waypoints and 
                not servo_active and 
                new_waypoint < len(goal_poses)):
                
                navigator.get_logger().info(f'到达第{new_waypoint+1}个航点，执行舵机动作')
                executed_waypoints.add(new_waypoint)
                servo_active = True
                
                # 异步执行舵机动作
                if servo_client.call_servo_action():
                    navigator.get_logger().info(f'第{new_waypoint+1}个航点舵机动作完成')
                else:
                    navigator.get_logger().error(f'第{new_waypoint+1}个航点舵机动作失败')
                
                # 动作完成后重置标志
                servo_active = False
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

if __name__ == '__main__':
    main()