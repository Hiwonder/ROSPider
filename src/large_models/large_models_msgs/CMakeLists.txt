cmake_minimum_required(VERSION 3.8)
project(large_models_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/AgentResult.msg"
    "msg/Transport.msg"
    "msg/Tools.msg"
    "srv/SetString.srv"
    "srv/SetModel.srv"
    "srv/SetContent.srv"
    "srv/SetInt32.srv"
    "srv/SetBox.srv"
    "srv/SetTransportObject.srv"
    "srv/SetTools.srv"
    DEPENDENCIES std_msgs std_srvs sensor_msgs builtin_interfaces
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

ament_package()
