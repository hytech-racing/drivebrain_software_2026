#include "drivebrain_app.hpp"
#include "ETHRecvComms.hpp"
#include "FoxgloveServer.hpp"
#include "InveterInterface.hpp"
#include "LidarInterface.hpp"
#include "SteeringInterface.hpp"
#include "MCAPLogger.hpp"
#include "hytech_msgs.pb.h"
#include "Telemetry.hpp"
#include "ControllerManager.hpp"
#include "DrivebrainControllerInterface.hpp"
#include "StateTracker.hpp"

#include <atomic>
#include <chrono>
#include <mcap/writer.hpp>
#include <memory>
#include <fmt/chrono.h>
#include <spdlog/spdlog.h>
#include <filesystem>
#include <stdexcept>

std::atomic<bool> running{true};

static constexpr size_t num_controllers = 1;

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
  core::MCAPLogger::create("recordings/", mcap::McapWriterOptions(""), _json_params_path);

  core::FoxgloveServer::create(_json_params_path);
  core::StateTracker::create();

  core::MCAPLogger::instance().open_new_mcap();
  core::MCAPLogger::instance().init_logging();

  spdlog::info("Constructed logging singletons");

  core::DrivebrainControllerInterface::create();

  spdlog::info("Constructed drivebrain controller interface");

// TODO: i really hate how this ifdef is fat smacked in the middle of the app
#ifndef HOOTL_ENABLED
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
#endif

  _inverter = InverterInterface::create(_dbc_path);
  _steering = SteeringInterface::create();
  _lidar = LidarInterface::create();
  _lidar->start();
  _sim_state_receiver = std::make_unique<SimStateReceiver>();
  _sim_state_receiver->start();

  _mode1 = std::make_shared<control::LoadCellTorqueController>();
  if (!_mode1->init()) {
    spdlog::error("Failed to initialize mode 1");
  }

  _estim_manager = std::make_shared<estimation::EstimatorManager>();

  std::array<std::shared_ptr<control::Controller<core::ControllerOutput, core::VehicleState>>, num_controllers> controllers{_mode1};

  ControllerManager<control::Controller<ControllerOutput, VehicleState>, num_controllers>::create(controllers);
  if (!ControllerManager<control::Controller<ControllerOutput, VehicleState>, num_controllers>::instance().init()) {
    throw std::runtime_error("Failed to initialize controller manager");
  }

  spdlog::info("Constructed controller manager");

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

  if(_loop_thread.joinable()) _loop_thread.join();
  _io_context.stop();
  if(_io_context_thread.joinable()) _io_context_thread.join();
  spdlog::info("Joined all threads");
}

void DrivebrainApp::_loop() {
  auto loop_time = 0.004;
  std::chrono::microseconds loop_time_ms((int) (loop_time * 1000000.0f));
  auto next_tick = std::chrono::steady_clock::now();

  auto desired_rpm_msg = std::make_shared<hytech::drivebrain_speed_set_input>();
  auto torque_limit_msg  = std::make_shared<hytech::drivebrain_torque_lim_input>();
  auto desired_torque_msg = std::make_shared<hytech::drivebrain_desired_torque_input>();

  while(running) {
    next_tick += loop_time_ms;

    auto state_and_validity = core::StateTracker::instance().get_latest_state_and_validity();
    _estim_manager->evaluate_all_estimators(state_and_validity.first);

    auto& controller_manager = ControllerManager<control::Controller<ControllerOutput, VehicleState>, num_controllers>::instance();
    auto out_struct = controller_manager.step_active_controller(state_and_validity.first);

    std::variant<core::SpeedControlOut, core::TorqueControlOut, std::monostate> cmd_out = out_struct.out;
    core::StateTracker::instance().set_previous_control_output(out_struct);

    if (state_and_validity.second) {
      if (const core::SpeedControlOut* sc = std::get_if<core::SpeedControlOut>(&cmd_out)) {
        desired_rpm_msg->set_drivebrain_set_rpm_fl(sc->desired_rpms.FL);
        desired_rpm_msg->set_drivebrain_set_rpm_fr(sc->desired_rpms.FR);
        desired_rpm_msg->set_drivebrain_set_rpm_rl(sc->desired_rpms.RL);
        desired_rpm_msg->set_drivebrain_set_rpm_rr(sc->desired_rpms.RR);

        torque_limit_msg->set_drivebrain_torque_fl(::abs(sc->torque_lim_nm.FL));
        torque_limit_msg->set_drivebrain_torque_fr(::abs(sc->torque_lim_nm.FR));
        torque_limit_msg->set_drivebrain_torque_rl(::abs(sc->torque_lim_nm.RL));
        torque_limit_msg->set_drivebrain_torque_rr(::abs(sc->torque_lim_nm.RR));

        core::log(desired_rpm_msg);
        core::log(torque_limit_msg);

        _inverter->send_rpm(*desired_rpm_msg);
        _inverter->send_torque_limit(*torque_limit_msg);

      } else if (const core::TorqueControlOut* tc = std::get_if<core::TorqueControlOut>(&cmd_out)) {
        // desired_torque_msg->set_drivebrain_torque_fl(tc->desired_torques_nm.FL);
        // desired_torque_msg->set_drivebrain_torque_fr(tc->desired_torques_nm.FR);
        // desired_torque_msg->set_drivebrain_torque_rl(tc->desired_torques_nm.RL);
        // desired_torque_msg->set_drivebrain_torque_rr(tc->desired_torques_nm.RR);

        desired_torque_msg->set_drivebrain_torque_fl(0.2);
        desired_torque_msg->set_drivebrain_torque_fr(0.2);
        desired_torque_msg->set_drivebrain_torque_rl(0.2);
        desired_torque_msg->set_drivebrain_torque_rr(0.2);

        core::log(desired_torque_msg);

        _inverter->send_torque(*desired_torque_msg);
      }

      // TODO remove later im too fucking lazy to care about this right now
      desired_torque_msg->set_drivebrain_torque_fl(0.1);
      desired_torque_msg->set_drivebrain_torque_fr(0.1);
      desired_torque_msg->set_drivebrain_torque_rl(015);
      desired_torque_msg->set_drivebrain_torque_rr(0.1);
      core::log(desired_torque_msg);
      _inverter->send_torque(*desired_torque_msg);

      if (_steering) {
        hytech_msgs::DBSteeringCommand steer_cmd;
        steer_cmd.set_desired_steering_angle_deg(-5.0f);
        _steering->send_steering(steer_cmd);
      }
    }

    std::tuple<std::string, bool> mcap_status = core::MCAPLogger::instance().status();
    auto mcap_info = std::make_shared<hytech_msgs::McapInfo>();
    mcap_info->set_current_mcap(std::get<0>(mcap_status));
    core::log(mcap_info);

    auto now = std::chrono::steady_clock::now();
    if(now > next_tick) {
      spdlog::warn("Loop overrun by {}", now-next_tick);
      next_tick = now;
    }

    std::this_thread::sleep_until(next_tick);
  }
}
