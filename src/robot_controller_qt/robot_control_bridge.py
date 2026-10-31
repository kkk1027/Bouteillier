#!/usr/bin/env python3
"""
改进版机器人控制桥接脚本
添加更好的命令处理和停止机制
"""

import sys
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time

class RobotControlBridge(Node):
    def __init__(self):
        super().__init__('robot_control_bridge')
        
        # 创建/cmd_vel话题的发布者
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 当前的速度消息
        self.current_twist = Twist()
        self.current_twist.linear.x = 0.0
        self.current_twist.linear.y = 0.0
        self.current_twist.linear.z = 0.0
        self.current_twist.angular.x = 0.0
        self.current_twist.angular.y = 0.0
        self.current_twist.angular.z = 0.0
        
        # 线程锁
        self.lock = threading.Lock()
        
        # 启动stdin读取线程
        self.read_thread = threading.Thread(target=self.read_stdin_loop, daemon=True)
        self.read_thread.start()
        
        # 创建定时器，20Hz发布速度
        self.timer = self.create_timer(0.05, self.publish_twist)  # 0.05秒 = 20Hz
        
        self.get_logger().info("🤖 机器人控制桥接器已启动 (改进版)")
    
    def read_stdin_loop(self):
        """从stdin循环读取命令"""
        while rclpy.ok():
            try:
                # 读取一行命令
                line = sys.stdin.readline()
                if not line:
                    break  # EOF
                
                line = line.strip()
                if line:
                    self.process_command(line)
            except Exception as e:
                self.get_logger().error(f"读取命令出错: {e}")
                time.sleep(0.01)  # 避免CPU占用过高
    
    def process_command(self, command_str):
        """处理从C++发送过来的命令"""
        try:
            command_str = command_str.strip().upper()  # 转为大写
            
            # 特殊停止命令
            if command_str in ["STOP", "STOP_NOW", "急停"]:
                with self.lock:
                    self.current_twist.linear.x = 0.0
                    self.current_twist.angular.z = 0.0
                
                # 立即发布停止命令
                stop_msg = Twist()
                self.publisher.publish(stop_msg)
                time.sleep(0.01)  # 短暂延迟
                self.publisher.publish(stop_msg)  # 再发一次确保停止
                
                self.get_logger().info("🛑 收到停止命令")
                return
            
            # 速度命令格式: linear_x,angular_z
            parts = command_str.split(',')
            if len(parts) == 2:
                try:
                    linear_x = float(parts[0])
                    angular_z = float(parts[1])
                    
                    # 更新当前速度
                    with self.lock:
                        self.current_twist.linear.x = linear_x
                        self.current_twist.angular.z = angular_z
                    
                    # 如果是停止命令（接近0的速度）
                    if abs(linear_x) < 0.001 and abs(angular_z) < 0.001:
                        stop_msg = Twist()
                        self.publisher.publish(stop_msg)
                        self.get_logger().debug("⏹️ 速度归零")
                    else:
                        self.get_logger().debug(f"🚀 设置速度: [{linear_x:.3f}, {angular_z:.3f}]")
                        
                except ValueError:
                    self.get_logger().warn(f"无效数字格式: {command_str}")
                    
        except Exception as e:
            self.get_logger().error(f"处理命令出错: {e}")
    
    def publish_twist(self):
        """定时发布速度消息"""
        with self.lock:
            # 只发布非零速度
            if abs(self.current_twist.linear.x) > 0.001 or abs(self.current_twist.angular.z) > 0.001:
                self.publisher.publish(self.current_twist)
            # 注意：不发布0速度，避免干扰真正的停止命令

def main():
    """主函数"""
    print("🚀 启动ROS2机器人控制桥接器 (改进版)...", file=sys.stderr)
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 创建节点
        node = RobotControlBridge()
        
        # 运行节点
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🔌 收到中断信号，正在关闭...", file=sys.stderr)
    except Exception as e:
        print(f"❌ 错误: {e}", file=sys.stderr)
    finally:
        # 清理
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("👋 桥接器已关闭", file=sys.stderr)

if __name__ == '__main__':
    main()
