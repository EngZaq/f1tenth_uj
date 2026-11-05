cmake_minimum_required(VERSION 3.8)
project(f1tenth_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Real physical integrated command selector and existing hardware-shadow explorer.
# Install the explorer and its ROS-free guard adjacent to preserve Python imports.
install(
  PROGRAMS
    scripts/physical_drive_mux.py
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/exploration/exploration_controller_v2.py
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/exploration/front_scan_guard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
