cmake_minimum_required(VERSION 3.5)
project(tfg_video_segmentation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

set(UIB_LIBS
        ${CMAKE_SOURCE_DIR}/library/libBasic.cpp
        ${CMAKE_SOURCE_DIR}/library/libColorSpace.cpp
        ${CMAKE_SOURCE_DIR}/library/libFlow.cpp
        ${CMAKE_SOURCE_DIR}/library/libImage.cpp
        ${CMAKE_SOURCE_DIR}/library/libImageFormatPM.cpp
        ${CMAKE_SOURCE_DIR}/library/libImageFormats.c
        ${CMAKE_SOURCE_DIR}/library/libMW.cpp
        ${CMAKE_SOURCE_DIR}/library/libPyramid.cpp)
set(OWN_LIBS
        ${CMAKE_SOURCE_DIR}/src/Track.cpp
        ${CMAKE_SOURCE_DIR}/src/Homography.cpp
        ${CMAKE_SOURCE_DIR}/src/ImageUtils.cpp)
set(EXTERNAL_LIBS
        fftw3
        fftw3f
        jpeg
        png
        tiff
        lapack
        blas)

add_executable(motionModel ${CMAKE_SOURCE_DIR}/src/src_motion_model.cpp)
add_library(segmentation ${OWN_LIBS})
add_library(UIB ${UIB_LIBS})

target_include_directories(motionModel
        PUBLIC ${CMAKE_SOURCE_DIR}/library)
target_link_libraries(motionModel segmentation)
target_link_libraries(segmentation UIB)
target_link_libraries(UIB ${EXTERNAL_LIBS})