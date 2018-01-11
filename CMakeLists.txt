cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

file(GLOB SOURCES
    "*.h"
    "*.cpp"
)
project(op_descriptor)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(OPENCV REQUIRED)

include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(${OPENCV_DEFINITIONS})


add_executable (op_main ${SOURCES})
target_link_libraries (op_main ${PCL_LIBRARIES})
