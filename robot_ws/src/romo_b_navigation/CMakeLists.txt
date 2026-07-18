cmake_minimum_required(VERSION 3.8)
project(romo_b_navigation)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(safe_command_heartbeat src/safe_command_heartbeat.cpp)
ament_target_dependencies(safe_command_heartbeat geometry_msgs rclcpp)

install(TARGETS safe_command_heartbeat DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees config launch DESTINATION share/${PROJECT_NAME})
ament_package()
