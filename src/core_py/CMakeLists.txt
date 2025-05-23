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

_ament_cmake_python_get_python_install_dir()

# Create Python package "wheeled_humanoid" from bindings
pybind11_add_module(
  wheeled_humanoid
  src/bindings.cpp
)
target_link_libraries(wheeled_humanoid PRIVATE wheeled_humanoid::wheeled_humanoid)

# Install Python package
install(
  TARGETS wheeled_humanoid
  DESTINATION ${PYTHON_INSTALL_DIR}
)

# Create Python package "wheeled_humanoid_py" with custom implementations and visualizations
ament_python_install_package(${PROJECT_NAME})

ament_package()
