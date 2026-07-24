cmake_minimum_required(VERSION 3.8)
project(residual_ground_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_library(residual_ground_filter_component SHARED
  src/point_cloud_filter_base.cpp
  src/residual_ground_filter_node.cpp
  src/uniform_grid_2d.cpp
  src/vertical_distribution.cpp)

target_compile_options(residual_ground_filter_component PRIVATE
  -Wall -Wextra -Wpedantic)

target_include_directories(residual_ground_filter_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  SYSTEM PRIVATE
    ${PCL_INCLUDE_DIRS})

target_link_libraries(residual_ground_filter_component
  ${PCL_LIBRARIES})

ament_target_dependencies(residual_ground_filter_component
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs)

rclcpp_components_register_node(residual_ground_filter_component
  PLUGIN "residual_ground_filter::ResidualGroundFilterComponent"
  EXECUTABLE residual_ground_filter_node
  EXECUTOR SingleThreadedExecutor)

install(TARGETS residual_ground_filter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES LICENSE README.md DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  pcl_conversions rclcpp rclcpp_components sensor_msgs tf2 tf2_ros tf2_sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vertical_distribution
    tests/test_vertical_distribution.cpp
    src/vertical_distribution.cpp)
  target_include_directories(test_vertical_distribution PRIVATE include)
  target_compile_options(test_vertical_distribution PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_uniform_grid_2d
    tests/test_uniform_grid_2d.cpp
    src/uniform_grid_2d.cpp)
  target_include_directories(test_uniform_grid_2d
    PRIVATE include
    SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_uniform_grid_2d ${PCL_LIBRARIES})
  target_compile_options(test_uniform_grid_2d PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_package()
