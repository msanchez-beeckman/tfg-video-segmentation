project (bgmm)

set(BGMM_LIBS
        ${PROJECT_SOURCE_DIR}/Homography.cpp
        ${PROJECT_SOURCE_DIR}/MotionModel.cpp)

add_library(${PROJECT_NAME} ${BGMM_LIBS})
target_link_libraries(${PROJECT_NAME}
        tfg::tracks
        Eigen3::Eigen
        ${OpenCV_LIBRARIES})
target_include_directories(${PROJECT_NAME}
        PUBLIC ${PROJECT_SOURCE_DIR})
add_library(tfg::bgmm ALIAS ${PROJECT_NAME})

add_executable(bgmm-bin ${PROJECT_SOURCE_DIR}/bgmm.cpp)
target_link_libraries(bgmm-bin
        tfg::bgmm
        ${OpenCV_LIBRARIES})
set_target_properties(bgmm-bin
        PROPERTIES OUTPUT_NAME bgmm)