cmake_minimum_required(VERSION 3.8)
project(f1tenth_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# GitHub-created scripts are mode 100644, and --symlink-install may preserve
# source file permissions: launch_ros Node requires executable bit and fails.
# Install as data, then start via explicit Python interpreter in launch.
# Keep front_scan_guard in the same folder as the explorer for Python imports.
install(
  FILES
    scripts/physical_drive_mux.py
    scripts/physical_path_guard.py
    scripts/physical_first_lap.py
    scripts/physical_map_finalizer.py
    scripts/map_coverage.py
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/exploration/exploration_controller_v2.py
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/exploration/front_scan_guard.py
  DESTINATION share/${PROJECT_NAME}/python_nodes
)

ament_package()
