
set(test_executable_name my_test)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_executable(${test_executable_name} ${SRCS})

target_include_directories(${test_executable_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# link_directories(${LIBRARY_OUTPUT_PATH})

target_link_libraries(
  ${test_executable_name} PUBLIC
  ${rclcpp_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
  ${std_msgs_LIBRARIES}
  devastator_perception_msgs__rosidl_typesupport_c
  devastator_perception_msgs__rosidl_typesupport_cpp
)


if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # uncomment the line when a copyright and license is not present in all source files
  # #set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # uncomment the line when this package is not in a git repo
  # #set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()
