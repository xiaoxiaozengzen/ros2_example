cmake_minimum_required(VERSION 3.15)
project(plus_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(plus_vendor
  VCS_URL plus
  VCS_TYPE path
  GLOBAL_HOOK
)

ament_package(
  CONFIG_EXTRAS "plus_vendor-extras.cmake"
)

