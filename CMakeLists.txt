cmake_minimum_required(VERSION 3.8)
project(radius_search_2d_outlier_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree search)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_library(radius_search_2d_outlier_filter_component SHARED
  src/filter.cpp
  src/radius_search_2d_outlier_filter_node.cpp
  src/vertical_distribution.cpp)

target_compile_options(radius_search_2d_outlier_filter_component PRIVATE
  -Wall -Wextra -Wpedantic)

target_include_directories(radius_search_2d_outlier_filter_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  SYSTEM PRIVATE
    ${PCL_INCLUDE_DIRS})

target_link_libraries(radius_search_2d_outlier_filter_component
  ${PCL_LIBRARIES})

ament_target_dependencies(radius_search_2d_outlier_filter_component
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs)

rclcpp_components_register_node(radius_search_2d_outlier_filter_component
  PLUGIN "radius_search_2d_outlier_filter::RadiusSearch2DOutlierFilterComponent"
  EXECUTABLE radius_search_2d_outlier_filter_node
  EXECUTOR SingleThreadedExecutor)

install(TARGETS radius_search_2d_outlier_filter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
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
endif()

ament_package()
