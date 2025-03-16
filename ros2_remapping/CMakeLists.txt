cmake_minimum_required(VERSION 3.5)
project(ros2_remapping)

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

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp)
set(SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
ament_auto_add_executable(${PROJECT_NAME}_exe ${SRCS})

ament_auto_add_executable(my_first_publish_exe ${CMAKE_CURRENT_SOURCE_DIR}/src/my_publish_first.cpp)
install(TARGETS my_first_publish_exe
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_add_executable(my_second_publish_exe ${CMAKE_CURRENT_SOURCE_DIR}/src/my_publish_second.cpp)
install(TARGETS my_second_publish_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
