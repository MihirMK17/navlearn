cmake_minimum_required(VERSION 3.8)
project(navlearn_assets)

find_package(ament_cmake REQUIRED)

# Worlds, their models, and the campaign occupancy maps install together: a map is
# generated from a specific world's collision geometry (see maps/README.md and the
# per-map PROVENANCE.md), so splitting them across packages would break the provenance
# chain the paper's localization numbers rest on.
#
# Upstream for the bookstore world and models is aws-robomaker (MIT-0), whose
# repositories were archived 2026-07-21 — vendoring is also the only way these stay
# reachable.
install(DIRECTORY worlds models maps
  DESTINATION share/${PROJECT_NAME}
)

# Registers this package's worlds/ and models/ on GZ_SIM_RESOURCE_PATH (and the
# Fortress-era IGN_GAZEBO_RESOURCE_PATH) at environment-source time. This is the whole
# integration contract: bumperbot_description's gazebo.launch.py searches that variable
# for worlds/<name>.world and appends it for model:// resolution, so the robot stack
# needs no compile-time or launch-time knowledge that this package exists.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/resource_paths.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
