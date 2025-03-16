cmake_minimum_required(VERSION 3.21)
project(rcl_node)

find_package(rcl REQUIRED)
find_package(rosidl_runtime_c REQUIRED)

link_directories(${CMAKE_BINARY_DIR}/my_rcl_test_msgs)

add_executable(rcl_publisher src/publisher.cpp)
target_include_directories(
  rcl_publisher
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rcl_INCLUDE_DIRS}
  ${rosidl_runtime_c_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/my_rcl_test_msgs/rosidl_generator_c
  ${CMAKE_BINARY_DIR}/my_rcl_test_msgs/rosidl_generator_cpp
)
target_link_libraries(rcl_publisher PUBLIC 
  ${rcl_LIBRARIES}
  ${rosidl_runtime_c_LIBRARIES}
  my_rcl_test_msgs__rosidl_generator_c
  my_rcl_test_msgs__rosidl_typesupport_c
  my_rcl_test_msgs__rosidl_typesupport_cpp
  pthread
)

add_executable(rcl_subscriber src/subscription.cpp)
target_include_directories(
  rcl_subscriber
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rcl_INCLUDE_DIRS}
  ${rosidl_runtime_c_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/my_rcl_test_msgs/rosidl_generator_c
  ${CMAKE_BINARY_DIR}/my_rcl_test_msgs/rosidl_generator_cpp
)
target_link_libraries(rcl_subscriber PUBLIC 
  ${rcl_LIBRARIES}
  ${rosidl_runtime_c_LIBRARIES}
  my_rcl_test_msgs__rosidl_generator_c
  my_rcl_test_msgs__rosidl_typesupport_c
  my_rcl_test_msgs__rosidl_typesupport_cpp
  pthread
)