cmake_minimum_required(VERSION 3.8)
project(arm_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)


# 添加下面的内容
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPose.srv"
  "srv/SetJoints.srv"
  DEPENDENCIES geometry_msgs
)






if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()




install(PROGRAMS
  scripts/arm_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# 如果需要安装整个Python包
install(DIRECTORY
  DESTINATION lib/${PROJECT_NAME}
)








ament_package()
