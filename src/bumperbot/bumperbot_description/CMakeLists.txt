cmake_minimum_required(VERSION 3.5)
project(bumperbot_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(scan_sanitizer src/scan_sanitizer.cpp)
ament_target_dependencies(scan_sanitizer rclcpp sensor_msgs)

target_compile_features(scan_sanitizer PUBLIC cxx_std_17)

install(
  TARGETS scan_sanitizer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch meshes models urdf rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
