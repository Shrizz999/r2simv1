cmake_minimum_required(VERSION 3.8)
project(arena_viz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch models rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
