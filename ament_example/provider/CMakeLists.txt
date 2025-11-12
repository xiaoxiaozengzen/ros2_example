cmake_minimum_required(VERSION 3.5)
project(provider)

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
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_auto_find_build_dependencies(REQUIRED nihao)

message("PROVIDER CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message("PROVIDER ENV{CMAKE_PREFIX_PATH}: $ENV{CMAKE_PREFIX_PATH}")
message("PROVDIER nihao_FOUND: ${nihao_FOUND}")
message("PROVIDER nihao_DIR: ${nihao_DIR}")
message("PROVIDER nihao_INCLUDE_DIRS: ${nihao_INCLUDE_DIRS}")
message("PROVIDER nihao_LIBRARIES: ${nihao_LIBRARIES}")
message("PROVIDER hello_FOUND: ${hello_FOUND}")

find_package(hello REQUIRED)
message("PROVIDER hello_FOUND: ${hello_FOUND}")
message("PROVIDER hello_DIR: ${hello_DIR}")
message("PROVIDER hello_INCLUDE_DIRS: ${hello_INCLUDE_DIRS}")
message("PROVIDER hello_LIBRARIES: ${hello_LIBRARIES}")

ament_auto_add_library(provider_lib SHARED
  src/provider_lib.cpp)
target_include_directories(provider_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${hello_INCLUDE_DIRS}
)
target_link_libraries(provider_lib ${hello_LIBRARIES})

ament_auto_add_executable(provider src/provider.cpp)
target_include_directories(provider PUBLIC ${hello_INCLUDE_DIRS})
target_link_libraries(provider ${hello_LIBRARIES})

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

ament_auto_package(INSTALL_TO_PATH INSTALL_TO_SHARE src)
