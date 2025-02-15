cmake_minimum_required(VERSION 3.8)
project(arm_controller)

# Declare dependencies
find_package(ament_cmake REQUIRED)
set(PACKAGE_DEPENDENCIES
  rclcpp
  Eigen3
  geometry_msgs
  sensor_msgs
  tf2_msgs
  tf2_ros
)

# Find dependencies
foreach(PKG ${PACKAGE_DEPENDENCIES})
  find_package(${PKG} REQUIRED)
endforeach()

# Library
add_library(${PROJECT_NAME} SHARED
  src/arm_kinematics.cpp
  src/controller.cpp
  src/ros_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})

# Executable
add_executable(${PROJECT_NAME}_node
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# Install
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
