cmake_minimum_required(VERSION 3.8)
project(taser_msgs)

# Declare dependencies
set(MSG_DEPENDENCIES
  geometry_msgs
)
set(PACKAGE_DEPENDENCIES
  ${MSG_DEPENDENCIES}
  ament_cmake
  rosidl_default_generators
)

# Find dependencies
foreach(PKG ${PACKAGE_DEPENDENCIES})
  find_package(${PKG} REQUIRED)
endforeach()

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveArms.action"
  "action/MoveBase.action"
  "srv/SetStates.srv"
  DEPENDENCIES ${MSG_DEPENDENCIES}
)

ament_package()
