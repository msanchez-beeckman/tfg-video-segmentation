project (utils)

set(UTIL_LIBS
        ${PROJECT_SOURCE_DIR}/ImageUtils.cpp
        ${PROJECT_SOURCE_DIR}/IoUtils.cpp)

add_library(${PROJECT_NAME} ${UTIL_LIBS})
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBRARIES})
target_include_directories(${PROJECT_NAME}
        PUBLIC ${PROJECT_SOURCE_DIR})
add_library(tfg::utils ALIAS ${PROJECT_NAME})