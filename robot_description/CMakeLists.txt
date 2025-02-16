cmake_minimum_required(VERSION 3.8)
project(robot_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install
install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
