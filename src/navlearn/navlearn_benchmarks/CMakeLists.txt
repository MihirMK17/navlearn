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
find_package(tf2_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

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

# ========= ScanRateGovernor (sensor-rate leg) =========
# Decimates the LiDAR stream to a commanded rate so the sensor-starvation leg varies
# exactly one thing. Lowering the simulated sensor's own rate would also change what
# Gazebo simulates and how the bridge is loaded, and would hide the rate in an edited
# description file where no provenance check would see it.
add_executable(scan_rate_governor src/scan_rate_governor.cpp)
target_include_directories(scan_rate_governor PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(scan_rate_governor rclcpp sensor_msgs)

# ========= Localization-eval trio (absorbed from navlearn_localization_eval) =========
# These three were a separate package with no declared dependency in either direction,
# yet benchmarks.launch.py composed all of them and episode_manager was the sole client
# of gz_set_pose_server's SetEntityPose service. The boundary existed only to be crossed;
# the executables keep their names, so launch composition and teardown are unchanged.

# Online TTC/TTR/ATE/RPE metrics against ground truth; writes the long-format
# localization CSV joined downstream by goal UUID.
add_executable(localization_metrics src/localization_metrics.cpp)
target_include_directories(localization_metrics PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})
ament_target_dependencies(localization_metrics
  rclcpp geometry_msgs sensor_msgs nav_msgs nav2_msgs navlearn_msgs tf2_msgs tf2_ros tf2
  unique_identifier_msgs)

# Republishes the Ignition ground-truth stream (/tf_gt) as the canonical
# /bumperbot/ground_truth_pose and the base_footprint_gt frame.
add_executable(ground_truth_publisher src/ground_truth_publisher.cpp)
target_include_directories(ground_truth_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})
ament_target_dependencies(ground_truth_publisher
  rclcpp geometry_msgs tf2_msgs tf2_ros)

# Bridges the SetEntityPose ROS service onto Ignition Transport for the kidnap teleport.
add_executable(gz_set_pose_server src/gz_set_pose_server.cpp)
target_include_directories(gz_set_pose_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gz_set_pose_server rclcpp geometry_msgs navlearn_msgs)
target_link_libraries(gz_set_pose_server
  ignition-transport11::ignition-transport11 ignition-msgs8::ignition-msgs8)

install(TARGETS
  episode_manager
  episode_manager_node
  control_metric
  trajectory_metric
  metrics_compiler
  collision_monitor
  scan_rate_governor
  localization_metrics
  ground_truth_publisher
  gz_set_pose_server
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
  # ament_flake8 ships its own config and never reads the repository's .flake8, so
  # without this it re-flags the E203 slice spacing that black itself emits.
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/flake8.cfg")
  ament_lint_auto_find_test_dependencies()

  # The pure-Python suites (nav2_stack parity gate, watchdog, forensics, models, maps,
  # seeds, and the claim analyses) live in navlearn_analysis, where colcon discovers
  # them wholesale. What remains here needs the BUILT nodes: the compiler contract
  # tests and the governor node test.
  find_package(ament_cmake_pytest REQUIRED)

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


  # Scan-rate governor node. The gtest above proves the keep rule; this proves the node:
  # correct topic wiring, the forwarded scan bit-identical to the input (a restamped
  # header would shift every measurement in AMCL's TF lookup), and a misconfigured rate
  # refused at startup rather than producing a cell that ran unstarved under a starved
  # label.
  ament_add_pytest_test(test_scan_rate_governor test/test_scan_rate_governor.py
    TIMEOUT 300
  )



  # Seed derivation. Asserts the two invariants the campaign's statistics rest on:
  # every (run, goal) draws a distinct condition, and all controllers draw identical
  # ones. Golden vectors pin splitmix64 itself — its constants are load-bearing, and
  # changing them would silently invalidate the reproducibility of prior runs.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navlearn_seed test/test_navlearn_seed.cpp)
  target_include_directories(test_navlearn_seed PRIVATE include)

  # Scan-rate decimation rule. Pins the two properties leg 6 rests on: the keep decision
  # is a pure function of the arrival index, so a rerun drops the same scans; and the
  # delivered rate is reported rather than assumed, so a starved cell cannot silently run
  # at a rate nobody requested.
  ament_add_gtest(test_scan_decimator test/test_scan_decimator.cpp)
  target_include_directories(test_scan_decimator PRIVATE include)

  # Continuous perturbation magnitude. The campaign made the curve the result instead of a
  # discarded calibration input, which only works if each goal's magnitude is derived from
  # the campaign seed rather than configured per cell. Pins that every goal draws a
  # distinct magnitude (the hardcoded-seed defect that invalidated the retired campaign's
  # confidence intervals had exactly this shape), that draws are reproducible, and that
  # coverage is even in the space the regression is fitted in.
  ament_add_gtest(test_magnitude_sampler test/test_magnitude_sampler.cpp)
  target_include_directories(test_magnitude_sampler PRIVATE include)

  # Bad-initialization offset sampling. The legacy sampler draws dx and dy independently
  # over a square, so its nominal range bounds each axis rather than the displacement and
  # the realised magnitude depends on direction -- fine for a categorical level, wrong for
  # a sweep whose independent variable IS the magnitude. Pins that the curve sampler
  # delivers exactly the commanded displacement, that bearings cover the circle so
  # severity is not confounded with direction, and documents the square sampler's
  # diagonal overshoot rather than leaving it to be rediscovered.
  ament_add_gtest(test_bad_init_offset test/test_bad_init_offset.cpp)
  target_include_directories(test_bad_init_offset PRIVATE include)

  # Kidnap yaw policy. PROTOCOL.md records the design as "yaw ... currently fixed -- one
  # variable moves"; the sampler drew yaw uniformly over the full circle instead, so the
  # declared independent variable (displacement) did nothing measurable and an undeclared
  # one did everything. Measured on the 2026-08-02 rpp arm: success 100% at 0-30 deg of
  # yaw change against 26.8% at 90-180 deg, while the same goals showed no trend at all
  # across displacement. Pins that the teleport preserves heading.
  ament_add_gtest(test_kidnap_yaw test/test_kidnap_yaw.cpp)
  target_include_directories(test_kidnap_yaw PRIVATE include)

  ament_add_gtest(test_exclusion_zone test/test_exclusion_zone.cpp)
  target_include_directories(test_exclusion_zone PRIVATE include)

endif()

ament_package()
