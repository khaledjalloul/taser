cmake_minimum_required(VERSION 3.8)
project(robot_launcher)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
