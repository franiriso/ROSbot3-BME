cmake_minimum_required(VERSION 3.8)
project(rosbot_3_bme)

# Set C++ standard to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include header files from /include/rosbot_3_bme
include_directories(include/rosbot_3_bme)

add_executable(test_node src/nodes/test_node.cpp)
add_executable(print_lidar_node src/nodes/print_lidar_node.cpp)
add_executable(obstacle_avoidance_node src/example1.cpp src/nodes/obstacle_avoidance_node.cpp)


# Link dependencies to targets
ament_target_dependencies(test_node rclcpp)
ament_target_dependencies(print_lidar_node rclcpp nav_msgs sensor_msgs)
ament_target_dependencies(obstacle_avoidance_node rclcpp nav_msgs sensor_msgs geometry_msgs)

# Install targets
install(TARGETS
  test_node
  print_lidar_node
  obstacle_avoidance_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()