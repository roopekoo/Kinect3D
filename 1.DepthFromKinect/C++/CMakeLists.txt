cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(KinectV2Depth)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(OpenCV REQUIRED)
find_package(freenect2 REQUIRED)
find_package(PCL 1.5 REQUIRED)

include_directories(${freenect2_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

add_executable(KinectV2Depth KinectV2Depth.cpp)

target_link_libraries(KinectV2Depth ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})