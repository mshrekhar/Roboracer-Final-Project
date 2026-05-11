cmake_minimum_required(VERSION 3.5)
project(pure_pursuit_reactive)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  rclpy 
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  Eigen3
  tf2_ros)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# --- EXECUTABLES ---

# 1. Pure Pursuit Node
add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
ament_target_dependencies(pure_pursuit_node 
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros
)

# 2. Velocity Obstacle Node (old — kept for reference)
add_executable(vo_filter_node src/velocity_obstacles.cpp)
ament_target_dependencies(vo_filter_node 
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros
)

# 3. Wall Follow Overtake Filter (new — replaces vo_filter_node)
add_executable(wall_follow_filter src/wall_follow_filter.cpp)
ament_target_dependencies(wall_follow_filter 
  rclcpp ackermann_msgs nav_msgs sensor_msgs std_msgs
)

# --- INSTALLATION ---

# 1. Install C++ executables
install(TARGETS
  pure_pursuit_node
  vo_filter_node
  wall_follow_filter
  DESTINATION lib/${PROJECT_NAME})

# 2. Install Launch files directly into share/pure_pursuit_reactive/
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

# 4. Install Python executables
install(PROGRAMS
  scripts/pure_pursuit_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()