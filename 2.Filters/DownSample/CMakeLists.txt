cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(DownSample)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (DownSample DownSample.cpp)
target_link_libraries (DownSample ${PCL_LIBRARIES})