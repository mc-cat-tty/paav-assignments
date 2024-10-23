cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 20)
set(CXX_STANDARD_REQUIRED on)

project(cluster_extraction)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (cluster_extraction src/cluster_extraction.cpp src/Renderer.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})
