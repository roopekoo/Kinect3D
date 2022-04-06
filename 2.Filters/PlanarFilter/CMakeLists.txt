cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(PlanarFilter)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (PlanarFilter PlanarFilter.cpp)
target_link_libraries (PlanarFilter ${PCL_LIBRARIES})