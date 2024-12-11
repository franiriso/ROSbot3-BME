cmake_minimum_required(VERSION 3.8)
project(rosbot_3_bme)

# Set C++ standard to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(test_node src/test_node.cpp)
add_executable(print_lidar_node src/print_lidar_node.cpp)
add_executable(avoid_obstacle_node src/avoid_obstacle_node.cpp)

ament_target_dependencies(test_node rclcpp)
ament_target_dependencies(print_lidar_node rclcpp nav_msgs sensor_msgs)
ament_target_dependencies(avoid_obstacle_node rclcpp nav_msgs sensor_msgs geometry_msgs)

install(TARGETS
  test_node
  print_lidar_node
  avoid_obstacle_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()