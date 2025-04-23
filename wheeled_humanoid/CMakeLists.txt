cmake_minimum_required(VERSION 3.8)
project(wheeled_humanoid)

# Declare dependencies
set(PACKAGE_DEPENDENCIES
  # casadi
  Eigen3
)

# Find dependencies
find_package(ament_cmake REQUIRED)
foreach(PKG ${PACKAGE_DEPENDENCIES})
  find_package(${PKG} REQUIRED)
endforeach()

# Library
add_library(${PROJECT_NAME} SHARED
  src/arm_controller.cpp
  src/arm_kinematics.cpp
  src/base_kinematics.cpp
  src/robot.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})

# Install
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Export
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
