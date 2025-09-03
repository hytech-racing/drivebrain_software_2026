cmake_minimum_required(VERSION 2.8.4)
project(libvncxx CXX)

set(SOURCE
        src/attitude.cpp
        src/compositedata.cpp
        src/conversions.cpp
        src/criticalsection.cpp
        src/dllvalidator.cpp
        src/error_detection.cpp
        src/event.cpp
        src/ezasyncdata.cpp
        src/memoryport.cpp
        src/packet.cpp
        src/packetfinder.cpp
        src/port.cpp
        src/position.cpp
        src/rtcmlistener.cpp
        src/rtcmmessage.cpp
        src/searcher.cpp
        src/sensors.cpp
        src/serialport.cpp
        src/thread.cpp
        src/types.cpp
        src/util.cpp
        src/utilities.cpp
        src/vntime.cpp
)

add_library(libvncxx ${SOURCE})
target_include_directories(libvncxx PUBLIC 
    $<INSTALL_INTERFACE:include/libvncxx>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    FILES_MATCHING PATTERN "*.h" 
)