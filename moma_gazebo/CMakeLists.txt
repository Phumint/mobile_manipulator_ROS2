cmake_minimum_required(VERSION 3.8)
project(moma_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config include launch src worlds models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/covariance_fixer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
