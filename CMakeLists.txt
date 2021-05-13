cmake_minimum_required(VERSION 3.5)
project(tfg-video-segmentation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(${CMAKE_SOURCE_DIR}/src/utils)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/flow)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/tracks)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/bgmm)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/random-walking)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/bvs)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/nlcv)