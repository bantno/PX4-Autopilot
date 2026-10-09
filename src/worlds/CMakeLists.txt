cmake_minimum_required(VERSION 3.8)
project(ahab_worlds)

find_package(ament_cmake REQUIRED)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_paths.dsv.in")

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
