cmake_minimum_required(VERSION 3.8)
project(navlearn_localization_eval)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(navlearn_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

add_executable(localization_metrics src/localization_metrics.cpp)
target_include_directories(localization_metrics PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})
ament_target_dependencies(localization_metrics
  rclcpp geometry_msgs sensor_msgs nav_msgs nav2_msgs navlearn_msgs tf2_msgs tf2_ros tf2 unique_identifier_msgs)

add_executable(ground_truth_publisher src/ground_truth_publisher.cpp)
target_include_directories(ground_truth_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})
ament_target_dependencies(ground_truth_publisher
  rclcpp geometry_msgs tf2_msgs tf2_ros)

add_executable(gz_set_pose_server src/gz_set_pose_server.cpp)
target_include_directories(gz_set_pose_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gz_set_pose_server rclcpp geometry_msgs navlearn_msgs)
target_link_libraries(gz_set_pose_server ignition-transport11::ignition-transport11 ignition-msgs8::ignition-msgs8)

install(TARGETS
  localization_metrics
  ground_truth_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  gz_set_pose_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
