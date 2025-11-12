cmake_minimum_required(VERSION 3.5)
project(my_ament_example)

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

# 编译得时候，会将所有得依赖包的路径都添加到CMAKE_PREFIX_PATH/AMENT_PREFIX_PATH环境变量中
# 编译结束后，source了install/setup.bash之后，printenv会打印出最新得CMAKE_PREFIX_PATH/AMENT_PREFIX_PATH
message("MY_AMENT_EXAMPLE ENV{CMAKE_PREFIX_PATH} = $ENV{CMAKE_PREFIX_PATH}")
message("MY_AMENT_EXAMPLE CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}")
message("MY_AMENT_EXAMPLE ENV{AMENT_PREFIX_PATH} = $ENV{AMENT_PREFIX_PATH}")
message("MY_AMENT_EXAMPLE AMENT_PREFIX_PATH = ${AMENT_PREFIX_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(hello REQUIRED)
find_package(world REQUIRED)

find_package(plus REQUIRED)

message("MY_AMENT_EXAMPLE plus_FOUND = ${plus_FOUND}")
message("MY_AMENT_EXAMPLE plus_DIR = ${plus_DIR}")
message("MY_AMENT_EXAMPLE plus_INCLUDE_DIRS = ${plus_INCLUDE_DIRS}")
message("MY_AMENT_EXAMPLE plus_LIBRARIES = ${plus_LIBRARIES}")

add_library(my_ament_example SHARED
  src/my_ament_example.cpp)

target_include_directories(my_ament_example
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${hello_INCLUDE_DIRS}
)

target_include_directories(my_ament_example PRIVATE
  ${world_INCLUDE_DIRS}
)

target_link_libraries(my_ament_example PUBLIC
  ${hello_LIBRARIES}
)

target_link_libraries(my_ament_example PRIVATE
  ${world_LIBRARIES}
)

target_include_directories(my_ament_example PUBLIC ${plus_INCLUDE_DIRS})
target_link_libraries(my_ament_example PRIVATE ${plus_LIBRARIES})

install(TARGETS my_ament_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(my_ament_example)
ament_export_dependencies(hello plus)

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
