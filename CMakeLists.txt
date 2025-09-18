cmake_minimum_required(VERSION 3.8)
project(taser_ros)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
  config
  launch
  robot_description
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
