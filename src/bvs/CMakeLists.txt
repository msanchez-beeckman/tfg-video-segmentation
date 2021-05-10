project (bvs)

set(BVS_LIBS
        ${PROJECT_SOURCE_DIR}/Grid.cpp)

set(MAXFLOW_LIBS
        ${CMAKE_SOURCE_DIR}/external/lib/maxflow-v3.04/graph.cpp
        ${CMAKE_SOURCE_DIR}/external/lib/maxflow-v3.04/maxflow.cpp)

add_library(${PROJECT_NAME} ${BVS_LIBS})
add_library(maxflow ${MAXFLOW_LIBS})
target_link_libraries(${PROJECT_NAME}
        tfg::tracks
        maxflow
        ${OpenCV_LIBRARIES})
target_include_directories(maxflow
        PUBLIC ${CMAKE_SOURCE_DIR}/external/lib)
target_include_directories(${PROJECT_NAME}
        PUBLIC ${PROJECT_SOURCE_DIR})
add_library(tfg::bvs ALIAS ${PROJECT_NAME})

add_executable(bvs-bin ${PROJECT_SOURCE_DIR}/bvs.cpp)
target_link_libraries(bvs-bin
        tfg::bvs
        ${OpenCV_LIBRARIES})
set_target_properties(bvs-bin
        PROPERTIES OUTPUT_NAME bvs)