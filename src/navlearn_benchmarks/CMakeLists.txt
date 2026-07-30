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
find_package(std_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

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
  rclcpp nav_msgs geometry_msgs unique_identifier_msgs navlearn_msgs sensor_msgs std_msgs)

# ========= CollisionMonitor (Gazebo Ignition adapter) =========
add_executable(collision_monitor src/collision_monitor.cpp)
target_include_directories(collision_monitor PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(collision_monitor
  rclcpp ros_gz_interfaces std_msgs)

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
  collision_monitor
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

# Python instrumentation run per benchmark episode by multi_run_harness.py.
install(PROGRAMS
  scripts/rate_monitor.py
  scripts/compute_sampler.py
  scripts/collision_positive_control.py
  scripts/costmap_corruption_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install empty benchmark_reports dir so the default csv_path/json_path from
# benchmarks.launch.py (FindPackageShare + "benchmark_reports/...") is always valid.
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/benchmark_reports
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # Disable ament_pep257: it enforces D213 (multi-line summary on the 2nd line),
  # which conflicts with the first-line-summary (D212) style used throughout these
  # scripts and is ignored by pydocstyle's own default convention. flake8 (run in
  # CI and respecting --max-line-length=100) covers real Python issues.
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # nav2_stack parity gate. Fails the build if the shipped navigation configuration
  # develops the drift the 2026-07-25 audit found — differing goal tolerances or speed
  # envelopes across controller arms, a per-controller costmap, an MPPI ObstaclesCritic
  # out of step with the inflation layer, or a multi-variable "single-variable" ablation.
  # Also proves each of those guards still fires, and that the harness refuses to run
  # against missing or stale run provenance.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_nav2_stack test/test_nav2_stack.py
    TIMEOUT 300
  )

  # Terminal pose report reaching the metrics CSV. Publishes the three messages
  # MetricsCompiler needs and inspects the resulting row, so the whole contract-to-column
  # path is covered without a simulator. Guards against a header/row column mismatch,
  # which shifts every downstream column invisibly, and against an absent ground-truth
  # measurement being written as 0.0, which would read as a perfect result.
  ament_add_pytest_test(test_terminal_pose test/test_terminal_pose.py
    TIMEOUT 300
  )

  # Kidnap reference pose reaching the metrics CSV. The secondary claim regresses
  # recovery on realised displacement, which is unrecoverable unless the robot's pose
  # immediately before the teleport is recorded; the fallback — the nominal commanded
  # band — is a bin label, and regressing on it handicaps distance against the predictor
  # it is being compared with. Guards the two silent modes: a defaulted reference at the
  # origin, which is real free space on the campaign map and would yield a plausible
  # fabricated displacement, and a destination written without yaw, which cannot be
  # scored for ambiguity at all.
  ament_add_pytest_test(test_kidnap_reference test/test_kidnap_reference.py
    TIMEOUT 300
  )

  # Map-derived destination ambiguity. Asserted against hand-checkable geometry rather
  # than golden values, because the failure mode that matters is a measure that is
  # self-consistent and wrong: a corridor must score as ambiguous and a landmark must
  # reduce ambiguity, or the number is not measuring identifiability whatever else it is
  # doing. Also pins the map axis convention, which if mirrored yields a perfectly
  # plausible field for a world that does not exist.
  ament_add_pytest_test(test_map_ambiguity test/test_map_ambiguity.py
    TIMEOUT 300
  )

  # Seed derivation. Asserts the two invariants the campaign's statistics rest on:
  # every (run, goal) draws a distinct condition, and all controllers draw identical
  # ones. Golden vectors pin splitmix64 itself — its constants are load-bearing, and
  # changing them would silently invalidate the reproducibility of prior runs.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navlearn_seed test/test_navlearn_seed.cpp)
  target_include_directories(test_navlearn_seed PRIVATE include)

  # The Python reference used by analysis, pinned to the same golden vectors so the two
  # implementations cannot silently diverge.
  ament_add_pytest_test(test_seed_reference test/test_seed_reference.py
    TIMEOUT 120
  )
endif()

ament_package()
