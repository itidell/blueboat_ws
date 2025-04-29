cmake_minimum_required(VERSION 3.8)
project(blueboat_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Install meshes - This block is correct
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
  FILES_MATCHING
  PATTERN "*.dae"
  PATTERN "*.stl"
)

# Install package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install other directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)

# REMOVED the redundant and incorrect install command for meshes that was here

ament_package()
