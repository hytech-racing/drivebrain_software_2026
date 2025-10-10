cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(drivebrain_software_2026)

include(FetchContent)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/cmake")

#################################
# Custom libs
#################################
add_subdirectory(drivebrain_core)
# add_subdirectory(drivebrain_comms)

#################################
# Find packages
#################################
find_package(foxglove-websocket REQUIRED)
find_package(Protobuf REQUIRED CONFIG)
find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)
find_package(mcap REQUIRED)

#################################
# Upstreams (non-conan)
#################################
FetchContent_Declare(
    HT_Proto 
    GIT_REPOSITORY https://github.com/hytech-racing/HT_proto.git
    GIT_TAG 5f61368

)
FetchContent_MakeAvailable(HT_Proto)

FetchContent_Declare(
    vn_driver_lib
    GIT_REPOSITORY https://github.com/hytech-racing/vn_driver_lib.git
    GIT_TAG 0d40803
)
FetchContent_MakeAvailable(vn_driver_lib)

#################################
# Linking to main app
#################################
add_executable(drivebrain
    drivebrain_simple_hil_test/src/debug_can.cpp
)

target_include_directories(drivebrain PUBLIC drivebrain_app/include)

target_link_libraries(drivebrain PUBLIC 
    foxglove-websocket::foxglove-websocket 
    protobuf::libprotobuf 
    boost::boost
    hytech_msgs_cpp_lib
    drivebrain_core
)