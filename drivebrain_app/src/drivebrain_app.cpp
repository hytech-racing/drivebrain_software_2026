#include "drivebrain_app.hpp"
#include "ETHRecvComms.hpp"
#include "FoxgloveServer.hpp"
#include "MCAPLogger.hpp"
#include "hytech_msgs.pb.h"
#include "Telemetry.hpp"
#include "ControllerManager.hpp"
#include "DrivebrainControllerInterface.hpp"
#include <StateTracker.hpp>
#include <atomic>
#include <chrono>
#include <mcap/writer.hpp>
#include <memory>
#include <fmt/chrono.h>
#include <spdlog/spdlog.h>
#include <filesystem>

std::atomic<bool> running{true};

void sig_handler(int signal) {
    if(signal == SIGINT) {
      spdlog::warn("Interrupted, stopping Drivebrain app");
      running = false;
    }
}

DrivebrainApp::DrivebrainApp(const std::string& json_param_path, const std::string& dbc_path)
  : _json_params_path(json_param_path)
  , _dbc_path(dbc_path) {}

DrivebrainApp::~DrivebrainApp() {
  running = false;

  core::MCAPLogger::instance().destroy();
  core::FoxgloveServer::instance().destroy();
  spdlog::info("Logging singletons safely destroyed");
}

void DrivebrainApp::run() {
  std::signal(SIGINT, sig_handler);

  // TODO: remove hardcoded paths
  core::MCAPLogger::create("recordings/", mcap::McapWriterOptions(""), _json_params_path);
  core::FoxgloveServer::create(_json_params_path);
  core::StateTracker::create();

  core::MCAPLogger::instance().open_new_mcap();
  core::MCAPLogger::instance().init_logging();

  spdlog::info("Constructed logging singletons");

  core::DrivebrainControllerInterface::create(); 

  spdlog::info("Constructed drivebrain controller interface");

  _acu_core_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::ACUCoreData>>(_io_context, 7777);
  _acu_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::ACUAllData>>(_io_context, 7766);
  _vcr_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::VCRData_s>>(_io_context, 9999);
  _vcf_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::VCFData_s>>(_io_context, 4444);

  bool vn_init_not_successful;
  _vn_driver = std::make_unique<comms::VNDriver>(_io_context, vn_init_not_successful);
  if (vn_init_not_successful) {
    spdlog::error("Failed to initialize vectornav driver");
  }


  spdlog::info("Initialized ethernet drivers");

  // CAN device names are defined in the drivebrain JSON config
  _telem_can = std::make_unique<comms::CANComms>(core::FoxgloveServer::instance().get_param<std::string>("telem_can_device").value(), _dbc_path);
  _aux_can = std::make_unique<comms::CANComms>(core::FoxgloveServer::instance().get_param<std::string>("aux_can_device").value(), _dbc_path);

  // Initialize controllers
  _controller1 = std::make_shared<control::LoadCellTorqueController>(); 
  if (!_controller1->init()) {
    spdlog::error("Failed to initialize controller");
  }

  spdlog::info("Initialized CAN drivers");

  _estim_manager = std::make_shared<estimation::EstimatorManager>();
  _estim_manager->handle_inits();
  spdlog::info("Constructed estimator manager");

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

    auto state_and_validity = core::StateTracker::instance().get_latest_state_and_validity();
    _estim_manager->evaluate_all_estimators(state_and_validity.first);

    std::shared_ptr<hytech::drivebrain_speed_set_input> speed_msg = std::make_shared<hytech::drivebrain_speed_set_input>(); 
    speed_msg->set_drivebrain_set_rpm_fl(1.0);
    speed_msg->set_drivebrain_set_rpm_fr(2.0);
    speed_msg->set_drivebrain_set_rpm_rl(4.0);
    speed_msg->set_drivebrain_set_rpm_rr(8.0);
    // _telem_can->send_message(speed_msg);

    std::tuple<std::string, bool> mcap_status = core::MCAPLogger::instance().status();
    std::string logile_name = std::get<0>(mcap_status);

    std::shared_ptr<hytech_msgs::McapInfo> mcap_info = std::make_shared<hytech_msgs::McapInfo>();
    mcap_info->set_current_mcap(logile_name);

    core::log(mcap_info);

    auto now = std::chrono::steady_clock::now();
    if(now > next_tick) {
      spdlog::warn("Loop overrun by {}", now-next_tick);
      next_tick = now;
    }

    std::this_thread::sleep_until(next_tick);
  }

}
