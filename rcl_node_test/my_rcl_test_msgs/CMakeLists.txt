project(my_rcl_test_msgs)

# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()
find_package(rcl REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_cmake REQUIRED)

file(GLOB_RECURSE MSGS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
set(msg_files ${MSGS})
message(STATUS "msg_files: ${msg_files}")
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  std_msgs
)