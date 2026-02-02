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
    drivebrain_comms
    GTest::gtest
)

file(COPY
  ${CMAKE_SOURCE_DIR}/tests/test_configs
  DESTINATION ${CMAKE_BINARY_DIR}
)
target_compile_definitions(drivebrain_test PRIVATE
    TEST_CONFIG_DIR="${CMAKE_BINARY_DIR}/test_configs"
)

include(GoogleTest)
gtest_discover_tests(drivebrain_test)

