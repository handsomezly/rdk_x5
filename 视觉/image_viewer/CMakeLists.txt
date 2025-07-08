cmake_minimum_required(VERSION 3.5)
project(image_viewer)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)

# Add include directories
include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare executable
# 添加可执行文件
add_executable(image_viewer_node
  src/image_viewer_node.cpp
  include/image_viewer/image_viewer_node.hpp) # 确保头文件被添加

# Add dependencies to the executable
ament_target_dependencies(
  image_viewer_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

# Install executable
install(TARGETS image_viewer_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
