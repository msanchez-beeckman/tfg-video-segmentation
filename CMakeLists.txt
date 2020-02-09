cmake_minimum_required(VERSION 3.5)
project(tfg_video_segmentation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(OWN_LIBS
        ${CMAKE_SOURCE_DIR}/src/Track.cpp
        ${CMAKE_SOURCE_DIR}/src/Mapping.cpp
        ${CMAKE_SOURCE_DIR}/src/TrackTable.cpp
        ${CMAKE_SOURCE_DIR}/src/Homography.cpp
        ${CMAKE_SOURCE_DIR}/src/ImageUtils.cpp
        ${CMAKE_SOURCE_DIR}/src/MotionModel.cpp
        ${CMAKE_SOURCE_DIR}/src/CmdParser.cpp)
# set(EXTERNAL_LIBS
#         fftw3
#         fftw3f
#         jpeg
#         png
#         tiff
#         lapack
#         blas
#         )

add_executable(motionModel ${CMAKE_SOURCE_DIR}/src/src_motion_model.cpp)
add_library(segmentation ${OWN_LIBS})

#target_include_directories(motionModel
#        PUBLIC ${CMAKE_SOURCE_DIR}/library)
target_link_libraries(motionModel segmentation)
target_link_libraries(segmentation ${OpenCV_LIBRARIES})