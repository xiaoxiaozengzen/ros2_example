cmake_minimum_required(VERSION 3.5)
project(ros2hz)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(WITH_REDIS "Build with REDIS support" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(toml11 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}
  src/generic_subscription.cpp
  src/ros2hz.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${toml11_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  toml11::toml11
)

target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${WITH_REDIS}>:WITH_REDIS>)
ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  rcpputils
  rcutils
  rosbag2_cpp
  rosbag2_storage
  rclcpp
  ament_index_cpp
  # toml11
  # yaml-cpp
)

ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
  rcpputils
  rcutils
  rosbag2_cpp
  rosbag2_storage
  rclcpp
  ament_index_cpp
  # toml11
  # yaml-cpp
)


# if (WITH_REDIS)
#     target_link_libraries(${PROJECT_NAME}_exe toml11::toml11 hiredis::hiredis)
# else()
#     target_link_libraries(${PROJECT_NAME}_exe toml11::toml11)
# endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_package()
