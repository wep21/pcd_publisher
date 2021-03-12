cmake_minimum_required(VERSION 3.5)
project(pcd_publisher)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED QUIET COMPONENTS common io)

### Target executable
ament_auto_add_library(pcd_publisher SHARED
  src/pcd_publisher.cpp
)

target_link_libraries(pcd_publisher
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(pcd_publisher
  PLUGIN "PCDPublisher"
  EXECUTABLE pcd_publisher_node
)

ament_auto_package()

