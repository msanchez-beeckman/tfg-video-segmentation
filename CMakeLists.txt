cmake_minimum_required(VERSION 3.5)
project(tfg_video_segmentation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(COMMON_LIBS   
        ${CMAKE_SOURCE_DIR}/src/ImageUtils.cpp
        ${CMAKE_SOURCE_DIR}/src/CmdParser.cpp
        ${CMAKE_SOURCE_DIR}/src/Track.cpp
        ${CMAKE_SOURCE_DIR}/src/Mapping.cpp
        ${CMAKE_SOURCE_DIR}/src/TrackTable.cpp)

set(HOMOGRAPHY_LIBS
        ${CMAKE_SOURCE_DIR}/src/Homography.cpp
        ${CMAKE_SOURCE_DIR}/src/MotionModel.cpp)

set(RANDOMWALKER_LIBS
        ${CMAKE_SOURCE_DIR}/src/RandomWalker.cpp)

# set(EXTERNAL_LIBS
#         fftw3
#         fftw3f
#         jpeg
#         png
#         tiff
#         lapack
#         blas
#         )

add_library(common ${COMMON_LIBS})
target_link_libraries(common ${OpenCV_LIBRARIES})

##### LIBRARIES FOR BACKGROUND MOTION MODELING
add_library(homography ${HOMOGRAPHY_LIBS})
target_link_libraries(homography common)

add_executable(motionModel ${CMAKE_SOURCE_DIR}/src/src_motion_model.cpp)
#target_include_directories(motionModel
#        PUBLIC ${CMAKE_SOURCE_DIR}/library)
target_link_libraries(motionModel homography)


##### LIBRARIES FOR RANDOM WALKER
add_library(walking ${RANDOMWALKER_LIBS})
target_link_libraries(walking common)

add_executable(randomWalker ${CMAKE_SOURCE_DIR}/src/src_random_walker.cpp)
target_link_libraries(randomWalker walking)