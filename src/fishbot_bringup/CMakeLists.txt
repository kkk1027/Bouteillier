cmake_minimum_required(VERSION 3.8)
project(fishbot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(odom2tf src/odom2tf.cpp)
ament_target_dependencies(odom2tf
  rclcpp tf2  nav_msgs  geometry_msgs  tf2_ros
)

add_executable(imu_bias_remover src/imu_bias_remover.cpp)
ament_target_dependencies(imu_bias_remover
  rclcpp sensor_msgs nav_msgs
)

install(TARGETS
  odom2tf
  imu_bias_remover
  DESTINATION lib/${PROJECT_NAME})

# 安装 Python 模块
ament_python_install_package(${PROJECT_NAME})

# 安装 Python 脚本为可执行节点
install(PROGRAMS
  fishbot_bringup/scan_qos_bridge.py
  fishbot_bringup/bag_analyzer.py
  fishbot_bringup/save_map.py
  fishbot_bringup/calibrate_wheelbase.py
  fishbot_bringup/monitor_ekf.py
  fishbot_bringup/odom_covariance_injector.py
  fishbot_bringup/manual_mapping_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
launch
config
DESTINATION share/${PROJECT_NAME}
)

ament_package()
