cmake_minimum_required(VERSION 3.8)
project(romo_b_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(pointcloud_filter src/pointcloud_filter.cpp)
ament_target_dependencies(pointcloud_filter
  pcl_conversions rclcpp sensor_msgs tf2_ros tf2_sensor_msgs)
target_include_directories(pointcloud_filter PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pointcloud_filter ${PCL_LIBRARIES})

add_executable(imu_normalizer src/imu_normalizer.cpp)
ament_target_dependencies(imu_normalizer rclcpp sensor_msgs)
target_include_directories(imu_normalizer PRIVATE include)

add_executable(pcd_to_occupancy src/pcd_to_occupancy.cpp)
target_include_directories(pcd_to_occupancy PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_to_occupancy ${PCL_LIBRARIES})

install(TARGETS pointcloud_filter imu_normalizer pcd_to_occupancy DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME pcd_to_occupancy_smoke
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/check_occupancy.py
      $<TARGET_FILE:pcd_to_occupancy>
      ${CMAKE_CURRENT_SOURCE_DIR}/test/fixture.pcd
      ${CMAKE_CURRENT_BINARY_DIR}/fixture_map 0.05 0.10 1.80
      ${CMAKE_CURRENT_SOURCE_DIR}/test/fixture.g2o 15.0 0.171)
  add_executable(test_imu_normalization test/test_imu_normalization.cpp)
  ament_target_dependencies(test_imu_normalization sensor_msgs)
  target_include_directories(test_imu_normalization PRIVATE include)
  add_test(NAME imu_normalization COMMAND $<TARGET_FILE:test_imu_normalization>)
endif()

ament_package()
