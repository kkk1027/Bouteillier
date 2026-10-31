cmake_minimum_required(VERSION 3.16)
project(robot_controller_qt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============ 设置正确的ROS2路径 ============
set(CMAKE_PREFIX_PATH
    "${CMAKE_PREFIX_PATH}"
    "/opt/ros/humble"
    "/usr/lib/x86_64-linux-gnu/cmake/Qt5"
    CACHE STRING "CMake查找路径" FORCE
)

# 查找Qt
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Concurrent)

# 查找ROS2包 - 使用ament_cmake方式
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Concurrent Network)

set(HAVE_ROS2 TRUE)
add_definitions(-DHAVE_ROS2)

message(STATUS "找到ROS2包:")
message(STATUS "  rclcpp_FOUND: ${rclcpp_FOUND}")
message(STATUS "  geometry_msgs_FOUND: ${geometry_msgs_FOUND}")
message(STATUS "  nav_msgs_FOUND: ${nav_msgs_FOUND}")
message(STATUS "  std_msgs_FOUND: ${std_msgs_FOUND}")
message(STATUS "  tf2_ros_FOUND: ${tf2_ros_FOUND}")

# Qt自动处理
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 源文件
set(SOURCES
    src/main.cpp
    src/launch_manager.cpp
    src/manual_controller.cpp
    src/map_display_widget.cpp
    src/ros_map_bridge.cpp
    src/camera_widget.cpp
)

# 头文件
set(HEADERS
    include/launch_manager.hpp
    include/manual_controller.hpp
    include/map_display_widget.hpp
    include/ros_map_bridge.hpp
    include/camera_widget.hpp
)

# 创建可执行文件
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# 链接Qt库
target_link_libraries(${PROJECT_NAME}
    Qt5::Core
    Qt5::Widgets
    Qt5::Concurrent
    Qt5::Network
)

# 链接ROS2库 - 使用ament的方式
target_link_libraries(${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${tf2_LIBRARIES}
    ${tf2_geometry_msgs_LIBRARIES}


)

# 或者使用更现代的方式（如果上面不行，尝试这个）：
# target_link_libraries(${PROJECT_NAME}
#     rclcpp::rclcpp
#     geometry_msgs::geometry_msgs
#     nav_msgs::nav_msgs
#     std_msgs::std_msgs
#     tf2_ros::tf2_ros
#     tf2::tf2
# )

# 包含目录
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 设置输出目录
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# 使用ament自动处理依赖
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    geometry_msgs
    nav_msgs
    std_msgs
    tf2_ros
    tf2
    tf2_geometry_msgs
)

message(STATUS "构建配置完成")
