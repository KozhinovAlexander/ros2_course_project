cmake_minimum_required(VERSION 3.8)
project(catch_them_all_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
