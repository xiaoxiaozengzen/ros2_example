cmake_minimum_required(VERSION 3.5)
project(mcap_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_BUILD_TYPE "Debug")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_auto_add_executable(bag_recorder_exe ${CMAKE_SOURCE_DIR}/src/bag_recorder.cpp)
ament_auto_add_executable(data_generate_exe ${CMAKE_SOURCE_DIR}/src/data_generate.cpp)
ament_auto_add_executable(meta_data_exe ${CMAKE_SOURCE_DIR}/src/meta_data.cpp)
ament_auto_add_executable(reader_exe ${CMAKE_SOURCE_DIR}/src/reader.cpp)
ament_auto_add_executable(player_exe ${CMAKE_SOURCE_DIR}/src/player.cpp)

ament_auto_package()
