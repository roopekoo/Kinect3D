cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(distance_filter)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (distance_filter distance_filter.cpp)
target_link_libraries (distance_filter ${PCL_LIBRARIES})
