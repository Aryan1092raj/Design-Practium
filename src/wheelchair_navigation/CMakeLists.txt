cmake_minimum_required(VERSION 3.8)
project(wheelchair_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
# PATTERN excludes archive/: superseded versions are kept in the repo for
# rollback but must not shadow the active configs in the installed share dir.
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "archive" EXCLUDE
)

# Install behavior tree files
install(DIRECTORY
  behavior_tree
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
