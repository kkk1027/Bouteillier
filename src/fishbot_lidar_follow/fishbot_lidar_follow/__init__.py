"""fishbot_lidar_follow —— 单线激光雷达目标跟随。

模块划分：
    lidar_follow   核心节点（控制逻辑、状态机、卡尔曼、安全检查）
    tracker        卡尔曼滤波与目标跟踪器（可单独测试）
    geometry       点云提取与聚类（纯函数，无 ROS 依赖）
    mutual_exclusion  与导航/其他跟随功能的互斥锁
    sim_node       离线仿真节点（用假数据跑一遍，方便调参）
"""
