#include "drivebrain_app.hpp"
#include "ETHRecvComms.hpp"
#include "FoxgloveServer.hpp"
#include "MCAPLogger.hpp"
#include "hytech_msgs.pb.h"

#include <atomic>
#include <chrono>
#include <mcap/writer.hpp>
#include <memory>
#include <fmt/chrono.h>
#include <spdlog/spdlog.h>

std::atomic<bool> running{true};

void sig_handler(int signal) {
    if(signal == SIGINT) {
      spdlog::warn("Interrupted, stopping Drivebrain app");
      running = false;
    }
}

DrivebrainApp::DrivebrainApp(const std::string& json_param_path, const std::string& dbc_path)
  : _json_params_path(json_param_path)
  , _dbc_path(dbc_path)
{}

DrivebrainApp::~DrivebrainApp() {
  running = false;

  core::MCAPLogger::instance().destroy();
  core::FoxgloveServer::instance().destroy();
  spdlog::info("Logging singletons safely destroyed");
}

void DrivebrainApp::run() {
  std::signal(SIGINT, sig_handler);

  // TODO: remove hardcoded paths
  core::FoxgloveServer::create(_json_params_path);
  core::MCAPLogger::create("recordings/", mcap::McapWriterOptions(""), _json_params_path);
  core::MCAPLogger::instance().open_new_mcap("test_1.mcap");
  core::MCAPLogger::instance().init_logging();

  spdlog::info("Constructed logging singletons");

  _acu_core_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::ACUCoreData>>(_io_context, 7777);
  _acu_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::ACUAllData>>(_io_context, 7766);
  _vcr_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::VCRData_s>>(_io_context, 9999);
  _vcf_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::VCFData_s>>(_io_context, 4444);

  spdlog::info("Initialized ethernet drivers");

  // CAN device names are defined in the drivebrain JSON config
  _telem_can = std::make_unique<comms::CANComms>(core::FoxgloveServer::instance().get_param<std::string>("telem_can_device").value(), _dbc_path);
  _aux_can = std::make_unique<comms::CANComms>(core::FoxgloveServer::instance().get_param<std::string>("aux_can_device").value(), _dbc_path);

  spdlog::info("Initialized CAN drivers");

  running = true; 
  _io_context_thread = std::thread([this]() {
    try {
      _io_context.run();
    } catch (const std::exception& e) {
      spdlog::error("io_context error: {}", e.what());
    }
   });

  _loop_thread = std::thread([this]() { _loop(); });
  spdlog::info("Spawned threads");

  // TODO: spawn future usb driver threads
  
  // Join threads when loop thread finishes
  if(_loop_thread.joinable()) _loop_thread.join();
  _io_context.stop();
  if(_io_context_thread.joinable()) _io_context_thread.join();
  spdlog::info("Joined all threads");

}

void DrivebrainApp::_loop() {
  auto loop_time = 0.004;
  std::chrono::microseconds loop_time_ms((int) (loop_time * 1000000.0f));
  auto next_tick = std::chrono::steady_clock::now();

  while(running) {
    next_tick += loop_time_ms;

    // TODO: update vehicle state
    // TODO: send control requests
    spdlog::debug("yo mama");


    auto now = std::chrono::steady_clock::now();
    if(now > next_tick) {
      spdlog::warn("Loop overrun by {}", now-next_tick);
      next_tick = now;
    }

    std::this_thread::sleep_until(next_tick);
  }

}
