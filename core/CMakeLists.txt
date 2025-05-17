cmake_minimum_required(VERSION 3.8)
project(wheeled_humanoid)

# Declare dependencies
set(PACKAGE_DEPENDENCIES
  ament_cmake
  Clipper2
  Eigen3
  osqp
  OsqpEigen
)

# Find dependencies
foreach(PKG ${PACKAGE_DEPENDENCIES})
  find_package(${PKG} REQUIRED)
endforeach()

# Library
add_library(${PROJECT_NAME} SHARED
  src/arm/controller.cpp
  src/arm/kinematics.cpp
  src/base/controller.cpp
  src/base/kinematics.cpp
  src/base/path_planner.cpp
  src/base/utils.cpp
  src/robot.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}
  Clipper2::Clipper2
  Eigen3::Eigen
  osqp::osqp
  OsqpEigen::OsqpEigen
)

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

# Test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test
    test/base_control_test.cpp
    test/path_planner_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_package()
