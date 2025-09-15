cmake_minimum_required(VERSION 3.8)
project(taser)

# Declare dependencies
set(PACKAGE_DEPENDENCIES
  Boost
  Eigen3
  osqp
  OsqpEigen
  pybind11
)

# Find dependencies
foreach(PKG ${PACKAGE_DEPENDENCIES})
  find_package(${PKG} REQUIRED)
endforeach()
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/cpp/src/arm_motion/controller.cpp
  src/cpp/src/arm_motion/kinematics.cpp
  src/cpp/src/locomotion/kinematics.cpp
  src/cpp/src/navigation/controller.cpp
  src/cpp/src/navigation/geometry.cpp
  src/cpp/src/navigation/path_planner.cpp
  src/cpp/src/robot.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/cpp/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  osqp::osqp
  OsqpEigen::OsqpEigen
)

# Install library
install(
  DIRECTORY src/cpp/include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Create Python package "taser_cpp" from bindings
pybind11_add_module(
  ${PROJECT_NAME}_cpp
  src/cpp/py/bindings.cpp
)
target_link_libraries(${PROJECT_NAME}_cpp PRIVATE ${PROJECT_NAME})

# Update runtime library search path to point to the installed library
set_target_properties(${PROJECT_NAME}_cpp PROPERTIES
  INSTALL_RPATH
  "${CMAKE_INSTALL_PREFIX}/lib;/workspaces/isaacsim/kit/python/lib/python3.11/site-packages/lib"
)

# Install Python package
install(
  TARGETS ${PROJECT_NAME}_cpp
  LIBRARY DESTINATION ${Python3_SITEARCH}
  RUNTIME DESTINATION ${Python3_SITEARCH}
)

# Test
include(CTest) # enables BUILD_TESTING option
if(BUILD_TESTING)
  find_package(GTest REQUIRED) # expects GTest to be installed
  include(GoogleTest) # for gtest_discover_tests

  add_executable(${PROJECT_NAME}_test
    src/cpp/test/nav_loco_test.cpp
    src/cpp/test/path_planner_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
  )

  # Auto-register each TEST() in ctest
  gtest_discover_tests(${PROJECT_NAME}_test)
endif()
