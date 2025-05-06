cmake_minimum_required(VERSION 3.16)
project(wheeled_humanoid_py)

# Declare dependencies
set(PACKAGE_DEPENDENCIES
  ament_cmake
  ament_cmake_python
  pybind11
  wheeled_humanoid
)

# Find dependencies
foreach(PKG ${PACKAGE_DEPENDENCIES})
  find_package(${PKG} REQUIRED)
endforeach()
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Install Python package with original name "wheeled_humanoid" that will import wheeled_humanoid_py
ament_python_install_package(wheeled_humanoid)

# Create Python package with bindings
pybind11_add_module(
  ${PROJECT_NAME}
  src/bindings.cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE wheeled_humanoid::wheeled_humanoid)

# Install Python package
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION ${PYTHON_INSTALL_DIR}
)

ament_package()
