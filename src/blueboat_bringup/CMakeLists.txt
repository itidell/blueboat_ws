cmake_minimum_required(VERSION 3.8)
project(blueboat_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(launch_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(blueboat_description REQUIRED)  # Only keep for CMake packages
find_package(robot_state_publisher REQUIRED)
find_package(blueboat_localization REQUIRED)
find_package(blueboat_navigation REQUIRED)
# Remove these lines - they're Python packages
# find_package(blueboat_sensors REQUIRED)
# find_package(blueboat_hardware_interface REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
