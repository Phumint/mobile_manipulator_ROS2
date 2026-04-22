cmake_minimum_required(VERSION 3.8)
project(moma_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install Python executable
install(
  PROGRAMS
    scripts/joint_state_merger.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config folders
install(
  DIRECTORY
    launch
    config
    rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
