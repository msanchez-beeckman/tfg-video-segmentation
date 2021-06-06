project (rdwalking)

set(RANDOMWALKER_LIBS
        ${PROJECT_SOURCE_DIR}/RandomWalker.cpp)

add_library(${PROJECT_NAME} ${RANDOMWALKER_LIBS})
target_link_libraries(${PROJECT_NAME}
        tfg::tracks
        Eigen3::Eigen)
target_include_directories(${PROJECT_NAME}
        PUBLIC ${PROJECT_SOURCE_DIR})
add_library(tfg::rdwalking ALIAS ${PROJECT_NAME})

add_executable(random_walk ${PROJECT_SOURCE_DIR}/random_walk.cpp)
target_link_libraries(random_walk
        tfg::rdwalking
        ${OpenCV_LIBRARIES})