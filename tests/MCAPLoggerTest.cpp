#include <cassert>
#include<gtest/gtest.h>
#include <FoxgloveServer.hpp>
#include <MCAPLogger.hpp>
#include <memory>

class MCAPLoggerTest : public testing::Test {

protected:
  core::FoxgloveServer server;

  MCAPLoggerTest()
   : server("test_configs/fake_foxglove_params.json") {};
  
};

TEST_F(MCAPLoggerTest, ParamFetch) {
  ASSERT_EQ(server.get_param<int>("rpm_limit"), 10000);
}
