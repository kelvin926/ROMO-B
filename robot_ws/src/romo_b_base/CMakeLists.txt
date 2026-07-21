cmake_minimum_required(VERSION 3.8)
project(romo_b_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(romo_b_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

add_library(romo_b_protocol src/protocol.cpp)
target_include_directories(romo_b_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(romo_b_serial_bridge src/serial_bridge.cpp)
target_link_libraries(romo_b_serial_bridge
  romo_b_protocol Boost::system Threads::Threads)
ament_target_dependencies(romo_b_serial_bridge
  diagnostic_msgs geometry_msgs lifecycle_msgs nav_msgs rclcpp rclcpp_lifecycle romo_b_msgs
  sensor_msgs std_msgs std_srvs)

install(TARGETS romo_b_protocol romo_b_serial_bridge
  EXPORT export_romo_b_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_romo_b_protocol HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol romo_b_protocol)
endif()

ament_package()
