cmake_minimum_required(VERSION 3.5)
project(ros2bag_example)

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

ament_auto_add_executable(db3_file_write_exe src/db3_file_write.cpp)
ament_auto_add_executable(db3_file_parse_exe src/db3_file_parse.cpp)
ament_auto_add_executable(mcap_file_reader_exe src/mcap_file_reader.cpp)
ament_auto_add_executable(ros2_bag_play_exe src/ros2_bag_play.cpp)
ament_auto_add_executable(metadata_yaml_exe src/metadata_yaml.cpp)
ament_auto_add_executable(data_generate_exe src/data_generate.cpp)

ament_auto_package()
