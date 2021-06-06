project (bvs)

set(BVS_LIBS
        ${PROJECT_SOURCE_DIR}/Grid.cpp)

add_library(${PROJECT_NAME} ${BVS_LIBS})
target_link_libraries(${PROJECT_NAME}
        tfg::tracks
        ${OpenCV_LIBRARIES})
target_include_directories(${PROJECT_NAME}
        PUBLIC ${PROJECT_SOURCE_DIR})
add_library(tfg::bvs ALIAS ${PROJECT_NAME})

add_executable(bvs-bin ${PROJECT_SOURCE_DIR}/bvs.cpp)
target_link_libraries(bvs-bin
        tfg::bvs
        ${OpenCV_LIBRARIES})
set_target_properties(bvs-bin
        PROPERTIES OUTPUT_NAME bvs)