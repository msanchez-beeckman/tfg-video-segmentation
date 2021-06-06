project (tracks)

set(TRACK_LIBS
        ${PROJECT_SOURCE_DIR}/Mapping.cpp
        ${PROJECT_SOURCE_DIR}/Track.cpp
        ${PROJECT_SOURCE_DIR}/TrackTable.cpp)

set(TRACKING_LIBS
        ${PROJECT_SOURCE_DIR}/Tracking.cpp)

add_library(${PROJECT_NAME} ${TRACK_LIBS})
target_link_libraries(${PROJECT_NAME}
        tfg::utils
        ${OpenCV_LIBRARIES})
target_include_directories(${PROJECT_NAME}
        PUBLIC ${PROJECT_SOURCE_DIR})
add_library(tfg::tracks ALIAS ${PROJECT_NAME})

add_library(tracking ${TRACKING_LIBS})
target_link_libraries(tracking
        tfg::tracks
        ${OpenCV_LIBRARIES})
target_include_directories(tracking
        PUBLIC ${PROJECT_SOURCE_DIR})
add_library(tfg::tracking ALIAS tracking)

add_executable(continue_tracks ${PROJECT_SOURCE_DIR}/continue_tracks.cpp)
target_link_libraries(continue_tracks
        tfg::tracking
        ${OpenCV_LIBRARIES})