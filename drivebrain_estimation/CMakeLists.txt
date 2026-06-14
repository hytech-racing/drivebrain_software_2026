cmake_minimum_required(VERSION 3.10)

add_library(drivebrain_estimation INTERFACE)

target_include_directories(drivebrain_estimation INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(drivebrain_estimation INTERFACE
    drivebrain_core
)
