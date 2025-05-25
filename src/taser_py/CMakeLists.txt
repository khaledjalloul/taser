cmake_minimum_required(VERSION 3.16)
project(taser_py)

# Declare dependencies
set(PACKAGE_DEPENDENCIES
  ament_cmake
  ament_cmake_python
  pybind11
  taser
)

# Find dependencies
foreach(PKG ${PACKAGE_DEPENDENCIES})
  find_package(${PKG} REQUIRED)
endforeach()
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

_ament_cmake_python_get_python_install_dir()

# Create Python package "taser" from bindings
pybind11_add_module(
  taser
  src/bindings.cpp
)
target_link_libraries(taser PRIVATE taser::taser)

# Install Python package
install(
  TARGETS taser
  DESTINATION ${PYTHON_INSTALL_DIR}
)

# Create Python package "taser_py" with custom implementations and visualizations
ament_python_install_package(${PROJECT_NAME})

ament_package()
