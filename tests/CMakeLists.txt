#################################
# Unit Testing
#################################

add_executable(drivebrain_test
    main.cpp
    MCAPLoggerTest.cpp
)

target_link_libraries(drivebrain_test PUBLIC
    foxglove-websocket::foxglove-websocket 
    mcap::mcap
    drivebrain_core
    GTest::gtest
)

include(GoogleTest)
gtest_discover_tests(drivebrain_test)
