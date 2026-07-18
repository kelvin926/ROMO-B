cmake_minimum_required(VERSION 3.8)
project(romo_b_navigation)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(safe_command_heartbeat src/safe_command_heartbeat.cpp)
ament_target_dependencies(safe_command_heartbeat geometry_msgs rclcpp)

add_library(romo_b_transient_obstacle_layer SHARED src/transient_obstacle_layer.cpp)
target_include_directories(romo_b_transient_obstacle_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  romo_b_transient_obstacle_layer nav2_costmap_2d pluginlib rclcpp)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(TARGETS
  safe_command_heartbeat
  romo_b_transient_obstacle_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees config launch DESTINATION share/${PROJECT_NAME})
ament_package()
