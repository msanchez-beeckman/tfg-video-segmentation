project (nlcv)

set(NLCV_LIBS
        ${PROJECT_SOURCE_DIR}/Region.cpp
        ${PROJECT_SOURCE_DIR}/ConsensusVoter.cpp)

add_library(${PROJECT_NAME} ${NLCV_LIBS})
target_link_libraries(${PROJECT_NAME}
        tfg::utils
        Eigen3::Eigen
        ${OpenCV_LIBRARIES})
target_include_directories(${PROJECT_NAME}
        PUBLIC ${PROJECT_SOURCE_DIR})
add_library(tfg::nlcv ALIAS ${PROJECT_NAME})

add_executable(nlcv-bin ${PROJECT_SOURCE_DIR}/nlcv.cpp)
target_link_libraries(nlcv-bin
        tfg::nlcv
        ${OpenCV_LIBRARIES})
set_target_properties(nlcv-bin
        PROPERTIES OUTPUT_NAME nlcv)