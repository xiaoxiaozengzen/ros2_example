cmake_minimum_required(VERSION 3.21)
project(hello)

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

# 编译的时候会打印这个变量，但是printenv的时候不会打印
message("ENV{CMAKE_PREFIX_PATH} = $ENV{CMAKE_PREFIX_PATH}")
if(DEFINED CMAKE_PREFIX_PATH)
  message("CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}")
else()
  message("CMAKE_PREFIX_PATH is not defined")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
message("ament_cmake_DIR = ${ament_cmake_DIR}")
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_library(hello SHARED
  src/hello.cpp)

target_include_directories(hello
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS hello
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(hello)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
