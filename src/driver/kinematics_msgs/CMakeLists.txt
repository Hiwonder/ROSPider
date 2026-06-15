cmake_minimum_required(VERSION 3.8)
project(kinematics_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Euler.msg"
    "msg/FeetPositions.msg"
    "msg/LegJoints.msg"
    "msg/LegsJoints.msg"
    "msg/LegPosition.msg"
    "msg/Pose.msg"
    "msg/RunActionSet.msg"
    "msg/SetHead.msg"
    "msg/State.msg"
    "msg/TransformEuler.msg"
    "msg/Traveling.msg"
    "srv/PoseTransform.srv"
    "srv/SetPose1.srv"
    "srv/SetPose2.srv"
    "srv/SetInt64.srv"
    "srv/SetFloat64.srv"
    "srv/SetFloat64List.srv"
    "srv/SimpleMoving.srv"
    DEPENDENCIES geometry_msgs
 )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
