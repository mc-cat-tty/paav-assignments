cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(tracker CXX)

set(CMAKE_CXX_STANDARD 20)
set(CXX_STANDARD_REQUIRED on)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(PCL 1.2 REQUIRED)
find_package(Eigen3 REQUIRED)
#-------------------------------------------------------------------------------
# Build libraries
#-------------------------------------------------------------------------------
include_directories (   ${CMAKE_CURRENT_SOURCE_DIR}/include 
                        ${EIGEN3_INCLUDE_DIR}
                        ${PCL_INCLUDE_DIRS}
                    )

file(GLOB tracker-SRC   "src/tracker/KalmanFilter.cpp"
                        "src/tracker/Tracklet.cpp"
                        "src/tracker/Tracker.cpp"
                        "src/viewer/Renderer.cpp"
                        "src/CloudManager.cpp"
                        )
set(tracker-LIBS ${PCL_LIBRARIES} pthread)
add_library(tracker SHARED ${tracker-SRC})
target_link_libraries(tracker ${tracker-LIBS} )

#-------------------------------------------------------------------------------
# Build executables
#-------------------------------------------------------------------------------
add_executable(main demo/main.cpp)
target_link_libraries(main tracker)




