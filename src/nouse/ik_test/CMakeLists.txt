cmake_minimum_required(VERSION 3.5)
project(ik_test)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPose.srv"
  DEPENDENCIES std_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY ik_test
  DESTINATION lib/python3.10/site-packages
)

ament_package()