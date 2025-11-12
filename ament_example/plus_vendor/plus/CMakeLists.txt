cmake_minimum_required(VERSION 3.15)
project(plus)
 
if(NOT DEFINED BUILD_SHARED_LIBS)
  set(BUILD_SHARED_LIBS ON)
endif()

include(CMakePackageConfigHelpers)
    
add_library(plus src/plus.cpp)
target_include_directories(plus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
    
install(TARGETS plus EXPORT plusTargets)

# Export the plusTargets for use with find_package
install(EXPORT plusTargets
  FILE plusTargets.cmake
  DESTINATION lib/cmake/plus)

set(PACKAGE_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(PACKAGE_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib")
set(PACKAGE_LIBRARIES "plus")

configure_package_config_file(plusConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/plusConfig.cmake"
  INSTALL_DESTINATION lib/cmake/plus
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)
write_basic_package_version_file(
  plusConfigVersion.cmake
  VERSION 1.1.1
  COMPATIBILITY SameMajorVersion)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/plusConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/plusConfigVersion.cmake"
  DESTINATION lib/cmake/plus)
install(DIRECTORY include/ DESTINATION include)

