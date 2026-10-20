cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

# Default to C++14 or C++17 depending on Harmonic standards
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings to ensure code quality and safety standards
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(gazebo_ros REQUIRED)   # Uncomment if linking against Gazebo C++ API

# C++ node: US and lidar octomaps fusion (Bayes log-odds)
add_executable(octomap_fusion_node octomap_fusion_node.cpp)
ament_target_dependencies(octomap_fusion_node
  rclcpp
  octomap_msgs
  visualization_msgs
  nav_msgs
)
target_link_libraries(octomap_fusion_node octomap)

install(
  DIRECTORY launch urdf maps config worlds meshes scripts
  DESTINATION share/${PROJECT_NAME}
)

# Python
install(
  PROGRAMS
    scripts/robot_teleop.py
    scripts/imu_reader.py
    scripts/navigator.py
    #scripts/checkpoint_cmd.py # Deprecated
    scripts/checkpoint_cmd_v2.py
    #scripts/frontier_explorer.py # Unused
    scripts/map_saver_service.py
    #scripts/ultrasonic_fusion_node.py
    #scripts/ultrasonic_fusion_node_v2.py
    scripts/ultrasonic_fusion_node_v3.py
    scripts/endurance_test.py
    scripts/ultrasonic_3d_map_node.py
    scripts/lidar_cloud_node.py
    
  DESTINATION lib/${PROJECT_NAME}
)

# C++
install(
  TARGETS octomap_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Exclude copyright/license checks for rapid development; enable in production.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()