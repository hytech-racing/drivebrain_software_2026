cmake_minimum_required(VERSION 3.10)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)

add_library(drivebrain_estimation
    src/ekf.cpp
    src/ekf_manager.cpp
    src/math_utils.cpp
)

target_include_directories(drivebrain_estimation
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(drivebrain_estimation
    PUBLIC
        Eigen3::Eigen
        Boost::boost
    PRIVATE
        spdlog::spdlog
)