cmake_minimum_required(VERSION 3.10)

add_library(drivebrain_hardware
    src/InverterInterface.cpp
    src/LidarInterface.cpp
)

target_include_directories(drivebrain_hardware PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(drivebrain_hardware PUBLIC
    drivebrain_core
    drivebrain_comms
)
