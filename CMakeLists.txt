# cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(drivebrain_software_2026)

include(FetchContent)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/cmake")

#################################
# Custom libs
#################################
add_subdirectory(drivebrain_core)
add_subdirectory(drivebrain_comms)

#################################
# Find packages
#################################
find_package(foxglove-websocket REQUIRED)
find_package(Protobuf REQUIRED CONFIG)
find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)
find_package(mcap REQUIRED)
find_package(GTest REQUIRED)

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

FetchContent_Declare(
    HT_CAN
    URL "https://github.com/hytech-racing/HT_CAN/releases/download/231/drivebrain_can_lib.tar.gz"
)
FetchContent_MakeAvailable(HT_CAN)

#################################
# Linking to main app
#################################
add_executable(drivebrain
    drivebrain_app/src/main.cpp
)

target_include_directories(drivebrain PUBLIC drivebrain_app/include)

target_link_libraries(drivebrain PUBLIC 
    foxglove-websocket::foxglove-websocket 
    mcap::mcap
    protobuf::libprotobuf 
    boost::boost
    hytech_msgs_cpp_lib
    hytech_can_msgs_cpp_lib
    drivebrain_core
    drivebrain_comms
)

#################################
# Unit Testing
#################################

if(NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    add_subdirectory(tests)
endif()
