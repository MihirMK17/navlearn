cmake_minimum_required(VERSION 3.8)
project(navlearn_benchmarks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(navlearn_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_library(episode_manager SHARED src/episode_manager.cpp)
target_include_directories(episode_manager PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(episode_manager
  PRIVATE "EPISODE_MANAGER_CPP_BUILDING_DLL")
ament_target_dependencies(episode_manager
  "navlearn_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "nav2_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "std_srvs")
rclcpp_components_register_node(episode_manager
  PLUGIN "navlearn_benchmarks::EpisodeManager"
  EXECUTABLE episode_manager_node
)

# ========= ControlMetric (standalone node) =========
add_executable(control_metric src/control_metric.cpp)
target_include_directories(control_metric PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(control_metric
  rclcpp nav_msgs geometry_msgs sensor_msgs builtin_interfaces
  unique_identifier_msgs navlearn_msgs)

# ========= TrajectoryMetric (standalone node) =========
add_executable(trajectory_metric src/trajectory_metric.cpp)
target_include_directories(trajectory_metric PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trajectory_metric
  rclcpp nav_msgs geometry_msgs unique_identifier_msgs navlearn_msgs sensor_msgs)

# ========= MetricsCompiler (standalone node) =========
add_executable(metrics_compiler src/metrics_compiler.cpp)
target_include_directories(metrics_compiler PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(metrics_compiler
  rclcpp unique_identifier_msgs navlearn_msgs tf2 geometry_msgs)

install(TARGETS
  episode_manager
  episode_manager_node
  control_metric
  trajectory_metric
  metrics_compiler
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
