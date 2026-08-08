cmake_minimum_required(VERSION 3.8)
project(navlearn_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Interface deps used inside the .msg files
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlMetric.msg"
  "msg/EpisodeEvent.msg"
  "msg/TerminalPoseReport.msg"
  "msg/TrajectoryMetric.msg"
  "msg/KidnapEvent.msg"
  "srv/SetEntityPose.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
    unique_identifier_msgs
)

# Lint/testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
