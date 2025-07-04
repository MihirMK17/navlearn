cmake_minimum_required(VERSION 3.8)
project(bumperbot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)


include_directories(include)

add_library(pd_motion_planner SHARED src/pd_motion_planner.cpp)
ament_target_dependencies(pd_motion_planner rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs nav2_core nav2_costmap_2d nav2_util pluginlib)

add_library(pure_pursuit SHARED src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs nav2_core nav2_costmap_2d nav2_util pluginlib)

pluginlib_export_plugin_description_file(nav2_core motion_planner_plugins.xml)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  pd_motion_planner
  pure_pursuit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES motion_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs nav2_core nav2_costmap_2d nav2_util pluginlib)
ament_export_libraries(pd_motion_planner pure_pursuit)
ament_package()
