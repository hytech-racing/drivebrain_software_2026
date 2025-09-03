cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(drivebrain_software_2026)

include(FetchContent)

add_subdirectory(vn_driver_lib)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Custom libs
add_subdirectory(drivebrain_core)
add_subdirectory(drivebrain_comms)

# Find packages
find_package(foxglove-websocket REQUIRED)
find_package(Protobuf REQUIRED CONFIG)
find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)

add_subdirectory(hytech_proto)

# Main app
add_executable(app
    drivebrain_app/src/main.cpp
)

target_include_directories(app PUBLIC drivebrain_app/include)

target_link_libraries(app PUBLIC 
    foxglove-websocket::foxglove-websocket 
    protobuf::libprotobuf 
    boost::boost 
    drivebrain_comms
    drivebrain_core
    hytech_msgs_cpp_lib
    libvncxx
)