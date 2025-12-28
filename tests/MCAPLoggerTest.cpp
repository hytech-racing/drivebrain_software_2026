#include <cassert>
#include <cstdint>
#include <gtest/gtest.h>
#include <FoxgloveServer.hpp>
#include <MCAPLogger.hpp>

class MCAPLoggerTest : public testing::Test {

protected:
  core::FoxgloveServer server;

  MCAPLoggerTest()
   : server(std::string(TEST_CONFIG_DIR) + "/fake_foxglove_params.json") {};
};

TEST_F(MCAPLoggerTest, ParamFetch) {
  ASSERT_EQ(server.get_param<int64_t>("rpm_limit"), 10000);
  ASSERT_EQ(server.get_param<bool>("enable_logging"), true);
}
