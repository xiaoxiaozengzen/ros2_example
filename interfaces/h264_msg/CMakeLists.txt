cmake_minimum_required(VERSION 3.5)
project(foxglove_msgs)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_auto_find_build_dependencies()
file(GLOB_RECURSE msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
# ament_export_dependencies(rosidl_default_runtime)
ament_auto_package()