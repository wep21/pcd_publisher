cmake_minimum_required(VERSION 3.14)
project(pcd_publisher)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

### Dependencies
find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PCL REQUIRED QUIET COMPONENTS common io)

### Target executable
add_library(pcd_publisher SHARED
  src/pcd_publisher.cpp
  src/pcd_publisher.hpp
)

target_link_libraries(pcd_publisher
  message_filters::message_filters
  pcl_common
  pcl_io
  ${pcl_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
)

rclcpp_components_register_node(pcd_publisher
  PLUGIN "PCDPublisher"
  EXECUTABLE pcd_publisher_node
)

install(TARGETS
  pcd_publisher
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()

