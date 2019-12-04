cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(bb_pcl_view)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (bb_view bb_view.cpp)
target_link_libraries (bb_view ${PCL_LIBRARIES})

