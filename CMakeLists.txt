cmake_minimum_required(VERSION 3.5)
project(tfg_video_segmentation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(UTIL_LIBS
        ${CMAKE_SOURCE_DIR}/src/utils/ImageUtils.cpp)

set(TRACK_LIBS
        ${CMAKE_SOURCE_DIR}/src/tracks/Mapping.cpp
        ${CMAKE_SOURCE_DIR}/src/tracks/Track.cpp
        ${CMAKE_SOURCE_DIR}/src/tracks/TrackTable.cpp)

set(BGMM_LIBS
        ${CMAKE_SOURCE_DIR}/src/bgmotionmodel/Homography.cpp
        ${CMAKE_SOURCE_DIR}/src/bgmotionmodel/MotionModel.cpp)

set(BVS_LIBS
        ${CMAKE_SOURCE_DIR}/src/bilateralvidseg/Grid.cpp)

set(MAXFLOW_LIBS
        ${CMAKE_SOURCE_DIR}/external/lib/maxflow-v3.04/graph.cpp
        ${CMAKE_SOURCE_DIR}/external/lib/maxflow-v3.04/maxflow.cpp)

set(RANDOMWALKER_LIBS
        ${CMAKE_SOURCE_DIR}/src/randomwalker/RandomWalker.cpp)

set(NLCV_LIBS
        ${CMAKE_SOURCE_DIR}/src/consensusvoting/Region.cpp
        ${CMAKE_SOURCE_DIR}/src/consensusvoting/ConsensusVoter.cpp)

##### LIBRARIES THAT ARE COMM0N FOR TWO OR MORE VIDEO SEGMENTATION STRATEGIES
add_library(utils ${UTIL_LIBS})
target_link_libraries(utils
        ${OpenCV_LIBRARIES})
target_include_directories(utils
        PUBLIC ${CMAKE_SOURCE_DIR}/src/utils)

add_library(tracks ${TRACK_LIBS})
target_link_libraries(tracks
        utils)
target_include_directories(tracks
        PUBLIC ${CMAKE_SOURCE_DIR}/src/tracks)

##### LIBRARIES FOR BACKGROUND MOTION MODELING (Szeliski)
add_library(bgmm ${BGMM_LIBS})
target_link_libraries(bgmm
        tracks)
target_include_directories(bgmm
        PUBLIC ${CMAKE_SOURCE_DIR}/src/bgmotionmodel)

add_executable(motionModel ${CMAKE_SOURCE_DIR}/src/bgmotionmodel/src_motion_model.cpp)
target_link_libraries(motionModel
        bgmm)

##### LIBRARIES FOR BILATERAL VIDEO SEGMENTATION (Märki, used by Szeliski)
add_library(maxflow ${MAXFLOW_LIBS})
add_library(bvs ${BVS_LIBS})
target_link_libraries(bvs
        tracks
        maxflow)
target_include_directories(maxflow
        PUBLIC ${CMAKE_SOURCE_DIR}/external/lib)
target_include_directories(bvs
        PUBLIC ${CMAKE_SOURCE_DIR}/src/bilateralvidseg)

add_executable(bilateralVidSeg ${CMAKE_SOURCE_DIR}/src/bilateralvidseg/src_bilateral_vidseg.cpp)
target_link_libraries(bilateralVidSeg
        bvs)

##### LIBRARIES FOR RANDOM WALKER (Naveen)
add_library(rdwalking ${RANDOMWALKER_LIBS})
target_link_libraries(rdwalking
        tracks)
target_include_directories(rdwalking
        PUBLIC ${CMAKE_SOURCE_DIR}/src/randomwalker)

add_executable(randomWalker ${CMAKE_SOURCE_DIR}/src/randomwalker/src_random_walker.cpp)
target_link_libraries(randomWalker
        rdwalking)

##### LIBRARIES FOR CONSENSUS VOTING (Faktor)
add_library(nlcv ${NLCV_LIBS})
target_link_libraries(nlcv
        utils)
target_include_directories(nlcv
        PUBLIC ${CMAKE_SOURCE_DIR}/src/consensusvoting)

add_executable(consensusVoting ${CMAKE_SOURCE_DIR}/src/consensusvoting/src_consensus_voting.cpp)
target_link_libraries(consensusVoting
        nlcv)
