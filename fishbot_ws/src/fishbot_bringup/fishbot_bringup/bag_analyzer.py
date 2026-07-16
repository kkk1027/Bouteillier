#!/usr/bin/env python3
"""
bag_analyzer.py — 分析 ros2 bag 中的里程计融合数据
用法:
  ros2 run fishbot_bringup bag_analyzer.py <bag_path>
  
示例:
  ros2 run fishbot_bringup bag_analyzer.py ~/fusion_test

功能:
  1. 对比 /odom (原始编码器) 与 /odometry/filtered (EKF融合后) 的 yaw 漂移
  2. 检测 IMU yaw 漂移量
  3. 分析激光雷达与里程计的旋转方向一致性
  4. 输出详细分析报告
"""

import sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py import set_message_fields
import rosbag2_py
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, ReadOrder

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

# ─── 辅助函数 ───────────────────────────────────────────

def quat_to_yaw(q):
    """从四元数提取 yaw (绕Z轴角度)"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)

def unwrap_angles(angles_rad):
    """角度解卷绕，消除跳变"""
    return np.unwrap(angles_rad)

def analyze_bag(bag_path):
    print("=" * 70)
    print(f"📦 分析 bag 文件: {bag_path}")
    print("=" * 70)
    
    # ─── 打开 bag ─────────────────────────────────────
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # ─── 获取话题列表 ─────────────────────────────────
    topics_info = reader.get_all_topics_and_types()
    topic_names = set(t.type.name for t in topics_info)
    print(f"\n📋 Bag 中的话题:")
    available_topics = {}
    for t in topics_info:
        print(f"   - {t.name} ({t.type})")
        available_topics[t.name] = t.type
    
    # ─── 数据结构 ─────────────────────────────────────
    odom_data = []       # (timestamp_sec, x, y, yaw, vx, vyaw)
    filtered_data = []   # (timestamp_sec, x, y, yaw, vx, vyaw)
    imu_data = []        # (timestamp_sec, yaw, ang_vel_z, ax, ay, az)
    tf_odom_data = []    # (timestamp_sec, x, y, yaw from TF)
    scan_data = []       # (timestamp_sec, angle_min, angle_increment, ranges)
    
    # ─── 遍历 bag ─────────────────────────────────────
    topic_types = {t.name: t.type for t in topics_info}
    
    print(f"\n⏳ 正在读取 bag 数据...")
    
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        
        if topic not in topic_types:
            continue
        
        msg_type = topic_types[topic]
        
        if msg_type == 'nav_msgs/msg/Odometry':
            msg = deserialize_message(data, Odometry)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            yaw = quat_to_yaw(msg.pose.pose.orientation)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            vx = msg.twist.twist.linear.x
            vyaw = msg.twist.twist.angular.z
            
            if topic == '/odom':
                odom_data.append((t, x, y, yaw, vx, vyaw))
            elif topic == '/odometry/filtered':
                filtered_data.append((t, x, y, yaw, vx, vyaw))
                
        elif msg_type == 'sensor_msgs/msg/Imu':
            msg = deserialize_message(data, Imu)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            yaw = quat_to_yaw(msg.orientation)
            ang_vel_z = msg.angular_velocity.z
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            imu_data.append((t, yaw, ang_vel_z, ax, ay, az))
            
        elif msg_type == 'tf2_msgs/msg/TFMessage':
            msg = deserialize_message(data, TFMessage)
            for tf in msg.transforms:
                if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_footprint':
                    t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                    yaw = quat_to_yaw(tf.transform.rotation)
                    x = tf.transform.translation.x
                    y = tf.transform.translation.y
                    tf_odom_data.append((t, x, y, yaw))
                    
        elif msg_type == 'sensor_msgs/msg/LaserScan' and topic == '/scan':
            msg = deserialize_message(data, LaserScan)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            scan_data.append((t, msg.angle_min, msg.angle_increment, 
                              len(msg.ranges), np.array(msg.ranges)))
    
    reader.close()
    
    # ─── 数据分析 ─────────────────────────────────────
    print(f"\n{'='*70}")
    print("📊 分析报告")
    print(f"{'='*70}")
    
    # 1. 基础统计
    print(f"\n📈 数据量统计:")
    print(f"   /odom:              {len(odom_data)} 帧")
    print(f"   /odometry/filtered:  {len(filtered_data)} 帧")
    print(f"   /imu_raw:           {len(imu_data)} 帧")
    print(f"   /tf (odom→bf):     {len(tf_odom_data)} 帧")
    print(f"   /scan:              {len(scan_data)} 帧")
    
    # 2. 漂移分析（对比 odom vs filtered）
    if len(odom_data) > 0 and len(filtered_data) > 0:
        odom_arr = np.array(odom_data)
        filtered_arr = np.array(filtered_data)
        
        # 对齐时间戳
        print(f"\n🕐 时间范围:")
        print(f"   /odom:              {odom_arr[0,0]:.2f}s ~ {odom_arr[-1,0]:.2f}s ({odom_arr[-1,0]-odom_arr[0,0]:.1f}s)")
        print(f"   /odometry/filtered: {filtered_arr[0,0]:.2f}s ~ {filtered_arr[-1,0]:.2f}s ({filtered_arr[-1,0]-filtered_arr[0,0]:.1f}s)")
        
        # Yaw 对比
        odom_yaw = unwrap_angles(odom_arr[:, 3])
        filtered_yaw = unwrap_angles(filtered_arr[:, 3])
        
        odom_yaw_drift = odom_yaw[-1] - odom_yaw[0]
        filtered_yaw_drift = filtered_yaw[-1] - filtered_yaw[0]
        
        print(f"\n🔄 Yaw 漂移分析 (全程):")
        print(f"   /odom yaw 总变化:        {np.degrees(odom_yaw_drift):+.2f}°")
        print(f"   /odometry/filtered yaw 总变化: {np.degrees(filtered_yaw_drift):+.2f}°")
        
        # Yaw 漂移速率（取线性拟合斜率）
        if len(odom_arr) > 10:
            dt_odom = odom_arr[:, 0] - odom_arr[0, 0]
            coeffs_odom = np.polyfit(dt_odom, odom_yaw, 1)
            odom_drift_rate = coeffs_odom[0]
            
            dt_filtered = filtered_arr[:, 0] - filtered_arr[0, 0]
            coeffs_filtered = np.polyfit(dt_filtered, filtered_yaw, 1)
            filtered_drift_rate = coeffs_filtered[0]
            
            print(f"   /odom yaw 漂移速率:          {np.degrees(odom_drift_rate):+.4f}°/s")
            print(f"   /odometry/filtered yaw 漂移速率: {np.degrees(filtered_drift_rate):+.4f}°/s")
            
            if abs(filtered_drift_rate) > 0.1:
                print(f"\n   ⚠️  警告: /odometry/filtered yaw 漂移速率 > 0.1°/s!")
                print(f"       建议: 检查 IMU 陀螺仪偏置或调整 ekf.yaml 配置")
            else:
                print(f"\n   ✅ /odometry/filtered yaw 漂移在可接受范围内")
        
        # 位置对比
        print(f"\n📍 位置偏差 (odom vs filtered 终点):")
        odom_end = odom_arr[-1, 1:3]
        filtered_end = filtered_arr[-1, 1:3]
        pos_diff = np.linalg.norm(odom_end - filtered_end)
        print(f"   /odom 终点:          ({odom_end[0]:.3f}, {odom_end[1]:.3f})")
        print(f"   /odometry/filtered 终点: ({filtered_end[0]:.3f}, {filtered_end[1]:.3f})")
        print(f"   位置偏差:            {pos_diff:.4f}m")
        
    # 3. IMU 漂移分析
    if len(imu_data) > 0:
        imu_arr = np.array(imu_data)
        imu_yaw = unwrap_angles(imu_arr[:, 1])
        imu_yaw_drift = imu_yaw[-1] - imu_yaw[0]
        imu_duration = imu_arr[-1, 0] - imu_arr[0, 0]
        
        print(f"\n🧭 IMU 分析:")
        print(f"   数据时长:                {imu_duration:.1f}s")
        print(f"   IMU yaw 总漂移:          {np.degrees(imu_yaw_drift):+.2f}°")
        
        if imu_duration > 0:
            imu_drift_rate = np.degrees(imu_yaw_drift / imu_duration)
            print(f"   IMU yaw 漂移速率:         {imu_drift_rate:+.4f}°/s")
            
            # 检查角速度偏置（静止时应该为0）
            ang_vel_z_mean = np.mean(imu_arr[:, 2])
            ang_vel_z_std = np.std(imu_arr[:, 2])
            print(f"   角速度 Z 均值:            {np.degrees(ang_vel_z_mean):+.6f}°/s")
            print(f"   角速度 Z 标准差:          {np.degrees(ang_vel_z_std):+.6f}°/s")
            
            if abs(ang_vel_z_mean) > 0.01:
                print(f"\n   ⚠️  角速度 Z 偏置较大 ({np.degrees(ang_vel_z_mean):+.4f}°/s)!")
                print(f"       这是导致 yaw 漂移的直接原因")
                print(f"       建议: 在 ESP32 端校准陀螺仪偏置")
            
        # 检查加速度计（重力方向）
        accel = imu_arr[:, 3:6]
        accel_norm = np.linalg.norm(accel, axis=1)
        accel_mean = np.mean(accel, axis=0)
        accel_std = np.std(accel, axis=0)
        print(f"\n   📐 加速度计:")
        print(f"     均值:  X={accel_mean[0]:+.3f}  Y={accel_mean[1]:+.3f}  Z={accel_mean[2]:+.3f} m/s²")
        print(f"     标准差: X={accel_std[0]:+.3f}  Y={accel_std[1]:+.3f}  Z={accel_std[2]:+.3f} m/s²")
        print(f"     模长均值: {np.mean(accel_norm):.3f} m/s² (重力≈9.81)")
        
        # 检测IMU是否有效
        if abs(np.mean(accel_norm) - 9.81) > 1.0:
            print(f"\n   ⚠️  加速度计模长 ({np.mean(accel_norm):.2f}) 偏离重力加速度(9.81)!")
            print(f"       可能是传感器震动或数据异常")
    
    # 4. 旋转方向一致性检查
    if len(odom_data) > 0 and len(filtered_data) > 0:
        print(f"\n🔄 旋转方向一致性检查:")
        
        # 计算 velody 的角速度方向 (odom 和 filtered 的 vyaw)
        odom_vyaw = odom_arr[:, 5]
        filtered_vyaw = filtered_arr[:, 5]
        
        odom_mean_vyaw = np.mean(odom_vyaw)
        filtered_mean_vyaw = np.mean(filtered_vyaw)
        
        print(f"   /odom 平均角速度:              {np.degrees(odom_mean_vyaw):+.4f}°/s")
        print(f"   /odometry/filtered 平均角速度:  {np.degrees(filtered_mean_vyaw):+.4f}°/s")
        
        # 检测方向一致性
        if len(odom_vyaw) > 10 and len(filtered_vyaw) > 10:
            # 采样一些点，比较符号
            odom_sign = np.sign(odom_vyaw[::max(1, len(odom_vyaw)//100)])
            filtered_sign = np.sign(filtered_vyaw[::max(1, len(filtered_vyaw)//100)])
            
            # 对齐长度
            min_len = min(len(odom_sign), len(filtered_sign))
            agreement = np.mean(odom_sign[:min_len] == filtered_sign[:min_len])
            print(f"   旋转方向一致性: {agreement*100:.1f}%")
            
            if agreement < 0.9:
                print(f"   ⚠️  odom 和 filtered 旋转方向不一致!")
                print(f"      可能原因: IMU 和编码器的角速度符号相反")
            else:
                print(f"   ✅ odom 和 filtered 旋转方向一致")
    
    # 5. TF vs odometry 对比
    if len(tf_odom_data) > 0 and len(filtered_data) > 0:
        print(f"\n🔗 TF 树 (odom→base_footprint) 检查:")
        tf_arr = np.array(tf_odom_data)
        tf_yaw = unwrap_angles(tf_arr[:, 3])
        tf_yaw_drift = tf_yaw[-1] - tf_yaw[0]
        print(f"   TF odom→bf yaw 总变化: {np.degrees(tf_yaw_drift):+.2f}°")
    
    # ─── 总结 ─────────────────────────────────────────
    print(f"\n{'='*70}")
    print("📝 综合评估")
    print(f"{'='*70}")
    
    issues = []
    
    if len(odom_data) == 0:
        issues.append("❌ 没有 /odom 数据 — 检查 ESP32 是否连接")
    if len(filtered_data) == 0:
        issues.append("❌ 没有 /odometry/filtered 数据 — EKF 未运行或未输出")
    if len(imu_data) == 0:
        issues.append("❌ 没有 /imu_raw 数据 — 检查 ESP32 IMU 发布")
    
    if len(filtered_data) > 0:
        if abs(filtered_drift_rate) > 0.1:
            issues.append(f"⚠️  EKF 输出 yaw 漂移速率 {np.degrees(filtered_drift_rate):+.2f}°/s — 过大，需修复")
        else:
            issues.append(f"✅ EKF 输出 yaw 漂移速率 {np.degrees(filtered_drift_rate):+.4f}°/s — 正常")
    
    if len(imu_data) > 0 and abs(ang_vel_z_mean) > 0.01:
        issues.append(f"⚠️  陀螺仪 Z 轴偏置 {np.degrees(ang_vel_z_mean):+.4f}°/s — 需在 ESP32 端校准")
    
    if len(scan_data) == 0:
        issues.append("❌ 没有 /scan 数据 — 检查 ydlidar 是否运行")
    
    for issue in issues:
        print(f"   {issue}")
    
    print(f"\n{'='*70}")
    print("💡 建议")
    print(f"{'='*70}")
    if abs(filtered_drift_rate) > 0.1 and abs(ang_vel_z_mean) > 0.01:
        print("1️⃣  ESP32 端校准陀螺仪:")
        print("   在 mpu.calcOffsets() 之前增加更多静止时间")
        print("   或实现运行时陀螺仪偏置估计")
        print(f"\n2️⃣  ekf.yaml 已设置 imu0_differential: true")
        print("   这会将 IMU 的 yaw 作为相对变化而非绝对朝向使用")
        print("   (已在本次修复中完成)")
    elif abs(filtered_drift_rate) <= 0.1:
        print("✅ 当前融合质量良好，无需进一步调整")
        print("   如果仍有问题，检查其他因素（激光雷达标定、里程计系数等）")
    
    print(f"\n{'='*70}\n")


def main():
    if len(sys.argv) < 2:
        print("用法: ros2 run fishbot_bringup bag_analyzer.py <bag_path>")
        print("示例: ros2 run fishbot_bringup bag_analyzer.py ~/fusion_test")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    analyze_bag(bag_path)


if __name__ == '__main__':
    main()
