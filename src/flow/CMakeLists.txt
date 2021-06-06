project (flow)

add_executable(optical_flow_dis ${PROJECT_SOURCE_DIR}/optical_flow_dis.cpp)
target_link_libraries(optical_flow_dis
        ${OpenCV_LIBRARIES})
