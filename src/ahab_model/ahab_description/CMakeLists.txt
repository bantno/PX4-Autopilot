cmake_minimum_required(VERSION 3.8)
project(ahab_description)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_paths.dsv.in")

xacro_add_files(
  urdf/ahab.urdf.xacro
    INSTALL DESTINATION urdf
)

install(DIRECTORY
  models
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
