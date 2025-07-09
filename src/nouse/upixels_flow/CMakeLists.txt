cmake_minimum_required(VERSION 3.8)
project(upixels_flow)

# 使用C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

add_executable(upixels_node src/LC-302-GS.cpp)

ament_target_dependencies(upixels_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  upixels_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()