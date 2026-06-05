#include "drivebrain_app.hpp"

#include <fmt/chrono.h>
#include <spdlog/spdlog.h>

#include <StateTracker.hpp>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <mcap/writer.hpp>
#include <memory>
#include <stdexcept>

#include "ControllerManager.hpp"
#include "DrivebrainControllerInterface.hpp"
#include "ETHRecvComms.hpp"
#include "FoxgloveServer.hpp"
#include "MCAPLogger.hpp"
#include "MatlabModelAddHelper.hpp"
#include "Telemetry.hpp"
#include "htx_ekf/ekf_manager.hpp"
#include "hytech_msgs.pb.h"

std::atomic<bool> running{true};

void sig_handler(int signal)
{
    if (signal == SIGINT)
    {
        spdlog::warn("Interrupted, stopping Drivebrain app");
        running = false;
    }
}

double try_get_ekf_double_param(const std::string& name, double fallback)
{
    auto& server = core::FoxgloveServer::instance();

    if (const auto value = server.get_param<float>(name))
    {
        return static_cast<double>(*value);
    }

    if (const auto value = server.get_param<double>(name))
    {
        return *value;
    }

    if (const auto value = server.get_param<int>(name))
    {
        return static_cast<double>(*value);
    }

    spdlog::warn("Using fallback for missing EKF double param {} = {}", name,
                 fallback);
    return fallback;
}

std::size_t try_get_ekf_size_param(const std::string& name,
                                   std::size_t fallback)
{
    auto& server = core::FoxgloveServer::instance();

    if (const auto value = server.get_param<int>(name))
    {
        return static_cast<std::size_t>(*value);
    }

    spdlog::warn("Using fallback for missing EKF size param {} = {}", name,
                 fallback);
    return fallback;
}

bool try_get_ekf_bool_param(const std::string& name, bool fallback)
{
    auto& server = core::FoxgloveServer::instance();

    if (const auto value = server.get_param<bool>(name))
    {
        return *value;
    }

    spdlog::warn("Using fallback for missing EKF bool param {} = {}", name,
                 fallback);
    return fallback;
}

htx_ekf::EkfManagerConfig load_ekf_manager_config_from_params()
{
    htx_ekf::EkfManagerConfig config;

    config.init_std_pn =
        try_get_ekf_double_param("htx_ekf/init_std_pn", config.init_std_pn);
    config.init_std_pe =
        try_get_ekf_double_param("htx_ekf/init_std_pe", config.init_std_pe);
    config.init_std_yaw =
        try_get_ekf_double_param("htx_ekf/init_std_yaw", config.init_std_yaw);
    config.init_std_vx =
        try_get_ekf_double_param("htx_ekf/init_std_vx", config.init_std_vx);
    config.init_std_vy =
        try_get_ekf_double_param("htx_ekf/init_std_vy", config.init_std_vy);
    config.init_std_alpha = try_get_ekf_double_param("htx_ekf/init_std_alpha",
                                                     config.init_std_alpha);
    config.init_std_bg =
        try_get_ekf_double_param("htx_ekf/init_std_bg", config.init_std_bg);
    config.init_std_bax =
        try_get_ekf_double_param("htx_ekf/init_std_bax", config.init_std_bax);
    config.init_std_bay =
        try_get_ekf_double_param("htx_ekf/init_std_bay", config.init_std_bay);

    config.alpha_alignment_min_speed = try_get_ekf_double_param(
        "htx_ekf/alpha_alignment_min_speed", config.alpha_alignment_min_speed);

    config.gnss1_offset_x_vehicle_frd =
        try_get_ekf_double_param("htx_ekf/gnss1_offset_x_vehicle_frd",
                                 config.gnss1_offset_x_vehicle_frd);
    config.gnss1_offset_y_vehicle_frd =
        try_get_ekf_double_param("htx_ekf/gnss1_offset_y_vehicle_frd",
                                 config.gnss1_offset_y_vehicle_frd);
    config.gnss2_offset_x_vehicle_frd =
        try_get_ekf_double_param("htx_ekf/gnss2_offset_x_vehicle_frd",
                                 config.gnss2_offset_x_vehicle_frd);
    config.gnss2_offset_y_vehicle_frd =
        try_get_ekf_double_param("htx_ekf/gnss2_offset_y_vehicle_frd",
                                 config.gnss2_offset_y_vehicle_frd);

    config.proc.q_std_pn =
        try_get_ekf_double_param("htx_ekf/q_std_pn", config.proc.q_std_pn);
    config.proc.q_std_pe =
        try_get_ekf_double_param("htx_ekf/q_std_pe", config.proc.q_std_pe);
    config.proc.q_std_yaw =
        try_get_ekf_double_param("htx_ekf/q_std_yaw", config.proc.q_std_yaw);
    config.proc.q_std_vx =
        try_get_ekf_double_param("htx_ekf/q_std_vx", config.proc.q_std_vx);
    config.proc.q_std_vy =
        try_get_ekf_double_param("htx_ekf/q_std_vy", config.proc.q_std_vy);
    config.proc.q_std_alpha = try_get_ekf_double_param("htx_ekf/q_std_alpha",
                                                       config.proc.q_std_alpha);
    config.proc.q_std_bg =
        try_get_ekf_double_param("htx_ekf/q_std_bg", config.proc.q_std_bg);
    config.proc.q_std_bax =
        try_get_ekf_double_param("htx_ekf/q_std_bax", config.proc.q_std_bax);
    config.proc.q_std_bay =
        try_get_ekf_double_param("htx_ekf/q_std_bay", config.proc.q_std_bay);

    config.zero_vel_update_sigma = try_get_ekf_double_param(
        "htx_ekf/zero_vel_update_sigma", config.zero_vel_update_sigma);

    config.gnss1_speed_update_sigma_floor =
        try_get_ekf_double_param("htx_ekf/gnss1_speed_update_sigma_floor",
                                 config.gnss1_speed_update_sigma_floor);
    config.gnss1_vel_update_sigma_floor =
        try_get_ekf_double_param("htx_ekf/gnss1_vel_update_sigma_floor",
                                 config.gnss1_vel_update_sigma_floor);
    config.gnss1_pos_update_sigma_floor =
        try_get_ekf_double_param("htx_ekf/gnss1_pos_update_sigma_floor",
                                 config.gnss1_pos_update_sigma_floor);
    config.gnss2_speed_update_sigma_floor =
        try_get_ekf_double_param("htx_ekf/gnss2_speed_update_sigma_floor",
                                 config.gnss2_speed_update_sigma_floor);
    config.gnss2_vel_update_sigma_floor =
        try_get_ekf_double_param("htx_ekf/gnss2_vel_update_sigma_floor",
                                 config.gnss2_vel_update_sigma_floor);
    config.gnss2_pos_update_sigma_floor =
        try_get_ekf_double_param("htx_ekf/gnss2_pos_update_sigma_floor",
                                 config.gnss2_pos_update_sigma_floor);

    config.use_gnss_reported_uncertainty =
        try_get_ekf_bool_param("htx_ekf/use_gnss_reported_uncertainty",
                               config.use_gnss_reported_uncertainty);

    config.zero_lat_base_sigma = try_get_ekf_double_param(
        "htx_ekf/zero_lat_base_sigma", config.zero_lat_base_sigma);
    config.zero_lat_disabled_sigma = try_get_ekf_double_param(
        "htx_ekf/zero_lat_disabled_sigma", config.zero_lat_disabled_sigma);
    config.zero_lat_yaw_rate_gain = try_get_ekf_double_param(
        "htx_ekf/zero_lat_yaw_rate_gain", config.zero_lat_yaw_rate_gain);
    config.zero_lat_lateral_accel_gain =
        try_get_ekf_double_param("htx_ekf/zero_lat_lateral_accel_gain",
                                 config.zero_lat_lateral_accel_gain);

    config.zero_vel_confidence_level = try_get_ekf_double_param(
        "htx_ekf/zero_vel_confidence_level", config.zero_vel_confidence_level);
    config.zero_lat_velocity_confidence_level =
        try_get_ekf_double_param("htx_ekf/zero_lat_velocity_confidence_level",
                                 config.zero_lat_velocity_confidence_level);
    config.gnss_speed_magnitude_confidence_level = try_get_ekf_double_param(
        "htx_ekf/gnss_speed_magnitude_confidence_level",
        config.gnss_speed_magnitude_confidence_level);
    config.gnss_velocity_ned_confidence_level =
        try_get_ekf_double_param("htx_ekf/gnss_velocity_ned_confidence_level",
                                 config.gnss_velocity_ned_confidence_level);
    config.gnss_pos_ned_confidence_level =
        try_get_ekf_double_param("htx_ekf/gnss_pos_ned_confidence_level",
                                 config.gnss_pos_ned_confidence_level);

    config.imu_stationary_required_samples =
        try_get_ekf_size_param("htx_ekf/imu_stationary_required_samples",
                               config.imu_stationary_required_samples);
    config.gnss1_stationary_required_samples =
        try_get_ekf_size_param("htx_ekf/gnss1_stationary_required_samples",
                               config.gnss1_stationary_required_samples);
    config.gnss2_stationary_required_samples =
        try_get_ekf_size_param("htx_ekf/gnss2_stationary_required_samples",
                               config.gnss2_stationary_required_samples);

    config.stationary_gyro_z_threshold =
        try_get_ekf_double_param("htx_ekf/stationary_gyro_z_threshold",
                                 config.stationary_gyro_z_threshold);
    config.stationary_accel_threshold =
        try_get_ekf_double_param("htx_ekf/stationary_accel_threshold",
                                 config.stationary_accel_threshold);
    config.stationary_speed_threshold_m_s =
        try_get_ekf_double_param("htx_ekf/stationary_speed_threshold_m_s",
                                 config.stationary_speed_threshold_m_s);
    config.stationary_required_samples =
        try_get_ekf_size_param("htx_ekf/stationary_required_samples",
                               config.stationary_required_samples);

    config.zero_lat_min_speed = try_get_ekf_double_param(
        "htx_ekf/zero_lat_min_speed", config.zero_lat_min_speed);
    config.gnss_velocity_ned_min_speed =
        try_get_ekf_double_param("htx_ekf/gnss_velocity_ned_min_speed",
                                 config.gnss_velocity_ned_min_speed);

    config.gnss_stale_timeout_s = try_get_ekf_double_param(
        "htx_ekf/gnss_stale_timeout_s", config.gnss_stale_timeout_s);

    config.alpha_lock_required_samples =
        try_get_ekf_size_param("htx_ekf/alpha_lock_required_samples",
                               config.alpha_lock_required_samples);
    config.alpha_gate_yaw_rate_max_rad_s =
        try_get_ekf_double_param("htx_ekf/alpha_gate_yaw_rate_max_rad_s",
                                 config.alpha_gate_yaw_rate_max_rad_s);
    config.alpha_gate_lateral_accel_max_m_s2 =
        try_get_ekf_double_param("htx_ekf/alpha_gate_lateral_accel_max_m_s2",
                                 config.alpha_gate_lateral_accel_max_m_s2);
    config.alpha_lock_spread_max_rad = try_get_ekf_double_param(
        "htx_ekf/alpha_lock_spread_max_rad", config.alpha_lock_spread_max_rad);

    return config;
}

DrivebrainApp::DrivebrainApp(const std::string& json_param_path,
                             const std::string& dbc_path)
    : _json_params_path(json_param_path), _dbc_path(dbc_path)
{
}

DrivebrainApp::~DrivebrainApp()
{
    running = false;

    core::MCAPLogger::instance().destroy();
    core::FoxgloveServer::instance().destroy();
    spdlog::info("Logging singletons safely destroyed");
}

void DrivebrainApp::run()
{
    std::signal(SIGINT, sig_handler);

    core::MCAPLogger::create("recordings/", mcap::McapWriterOptions(""),
                             _json_params_path);
    core::FoxgloveServer::create(_json_params_path);
    core::StateTracker::create();

    const htx_ekf::EkfManagerConfig ekf_config =
        load_ekf_manager_config_from_params();

    _ekf_manager = std::make_shared<htx_ekf::EkfManager>(ekf_config);

    spdlog::info("Constructed EKF manager");

    _last_ekf_reset_request = core::FoxgloveServer::instance()
                                  .get_param<int>("htx_ekf/reset_request_id")
                                  .value_or(0);

    _ekf_reset_param_conn =
        core::FoxgloveServer::instance().register_param_callback(
            [this](const std::unordered_map<std::string, core::DBParam>&
                       params) mutable
            {
                spdlog::info(
                    "*********I is in the EKF reset callback*********");
                const auto it = params.find("htx_ekf/reset_request_id");

                if (it == params.end())
                {
                    spdlog::info("EKF callback: reset key missing this cycle");
                    return;
                }

                int request_id = _last_ekf_reset_request;
                bool parsed = false;

                if (const auto as_int = std::get_if<int>(&it->second))
                {
                    request_id = *as_int;
                    parsed = true;
                }
                else if (const auto as_float = std::get_if<float>(&it->second))
                {
                    request_id = static_cast<int>(*as_float);
                    parsed = true;
                }
                if (!parsed)
                {
                    spdlog::warn(
                        "Ignoring htx_ekf/reset_request_id with unsupported "
                        "type");
                    return;
                }
                if (request_id < 0)
                {
                    spdlog::warn(
                        "Ignoring negative htx_ekf/reset_request_id={}",
                        request_id);
                    return;
                }

                spdlog::info("EKF hard reset request id: {}", request_id);
                spdlog::info("EKF hard reset last request id: {}",
                             _last_ekf_reset_request);

                if (request_id != _last_ekf_reset_request)
                {
                    spdlog::warn("EKF hard reset requested: {} -> {}",
                                 _last_ekf_reset_request, request_id);
                    if (_ekf_manager)
                    {
                        _ekf_manager->hard_reset();
                    }
                    _last_ekf_reset_request = request_id;
                }
            });

    spdlog::info("Registered EKF reset param callback");

    core::MCAPLogger::instance().open_new_mcap();
    core::MCAPLogger::instance().init_logging();

    spdlog::info("Constructed logging singletons");

    core::DrivebrainControllerInterface::create();

    spdlog::info("Constructed drivebrain controller interface");

    _acu_core_eth_driver =
        std::make_unique<comms::ETHRecvComms<hytech_msgs::ACUCoreData>>(
            _io_context, 7777);
    _acu_eth_driver =
        std::make_unique<comms::ETHRecvComms<hytech_msgs::ACUAllData>>(
            _io_context, 7766);
    _vcr_eth_driver =
        std::make_unique<comms::ETHRecvComms<hytech_msgs::VCRData_s>>(
            _io_context, 9999);
    _vcf_eth_driver =
        std::make_unique<comms::ETHRecvComms<hytech_msgs::VCFData_s>>(
            _io_context, 4444);

    bool vn_init_not_successful;
    _vn_driver = std::make_unique<comms::VNDriver>(
        _io_context, vn_init_not_successful, _ekf_manager);
    if (vn_init_not_successful)
    {
        spdlog::error("Failed to initialize vectornav driver");
    }

    spdlog::info("Initialized ethernet drivers");

    // CAN device names are defined in the drivebrain JSON config
    _telem_can = std::make_unique<comms::CANComms>(
        core::FoxgloveServer::instance()
            .get_param<std::string>("telem_can_device")
            .value(),
        _dbc_path);
    _aux_can = std::make_unique<comms::CANComms>(
        core::FoxgloveServer::instance()
            .get_param<std::string>("aux_can_device")
            .value(),
        _dbc_path);
    spdlog::info("Initialized CAN drivers");

    // Initialize controllers
    const size_t num_controllers = 1 + matlab_model_gen::num_controllers;
    _mode1 = std::make_shared<control::LoadCellTorqueController>();
    if (!_mode1->init())
    {
        spdlog::error("Failed to initialize mode 1");
    }

    // Estimator Manager
    _estim_manager = std::make_shared<estimation::EstimatorManager>();
    _estim_manager->handle_inits();
    spdlog::info("Constructed estimator manager");

    std::array<std::shared_ptr<control::Controller<core::ControllerOutput,
                                                   core::VehicleState>>,
               num_controllers>
        controllers{_mode1};
    auto _gend_controllers =
        matlab_model_gen::create_controllers(_estim_manager);
    if (_gend_controllers.size() + 1 != controllers.size())
    {
        throw std::runtime_error(
            "Failed to initialize matlab generated controllers! Wrong vector "
            "size!");
    }
    std::copy(_gend_controllers.begin(), _gend_controllers.end(),
              controllers.begin() + 1);

    // Create controller manager instance
    ControllerManager<control::Controller<ControllerOutput, VehicleState>,
                      num_controllers>::create(controllers);
    if (!ControllerManager<control::Controller<ControllerOutput, VehicleState>,
                           num_controllers>::instance()
             .init())
    {
        throw std::runtime_error("Failed to initialize controller manager");
    }

    spdlog::info("Constructed controller manager");

    running = true;
    _io_context_thread = std::thread(
        [this]()
        {
            try
            {
                _io_context.run();
            }
            catch (const std::exception& e)
            {
                spdlog::error("io_context error: {}", e.what());
            }
        });

    _loop_thread = std::thread(
        [this]()
        {
            try
            {
                _loop();
            }
            catch (const std::exception& e)
            {
                spdlog::error("_loop threw: {}", e.what());
            }
            catch (...)
            {
                spdlog::error("_loop threw unknown exception");
            }
            spdlog::error("_loop thread exiting, running={}", running.load());
        });
    spdlog::info("Spawned threads");

    // Join threads when loop thread finishes
    if (_loop_thread.joinable()) _loop_thread.join();
    _io_context.stop();
    if (_io_context_thread.joinable()) _io_context_thread.join();
    spdlog::info("Joined all threads");
}

void DrivebrainApp::_loop()
{
    auto loop_time = 0.004;
    std::chrono::microseconds loop_time_ms((int)(loop_time * 1000000.0f));
    auto next_tick = std::chrono::steady_clock::now();

    auto desired_rpm_msg =
        std::make_shared<hytech::drivebrain_speed_set_input>();
    auto torque_limit_msg =
        std::make_shared<hytech::drivebrain_torque_lim_input>();
    auto desired_torque_msg =
        std::make_shared<hytech::drivebrain_desired_torque_input>();

    while (running)
    {
        // spdlog::info("tick: start");

        next_tick += loop_time_ms;

        // spdlog::info("tick: get_state");

        auto state_and_validity =
            core::StateTracker::instance().get_latest_state_and_validity();

        // spdlog::info("tick: evaluate_estimators");

        _estim_manager->evaluate_all_estimators(state_and_validity.first);

        // spdlog::info("tick: step_controller");

        auto& controller_manager = ControllerManager<
            control::Controller<ControllerOutput, VehicleState>,
            1 + matlab_model_gen::num_controllers>::instance();
        auto out_struct =
            controller_manager.step_active_controller(state_and_validity.first);

        // spdlog::info("tick: variant_branch");

        std::variant<core::SpeedControlOut, core::TorqueControlOut,
                     std::monostate>
            cmd_out = out_struct.out;
        core::StateTracker::instance().set_previous_control_output(out_struct);

        bool state_is_valid = state_and_validity.second;

        // spdlog::info("tick: enter_send_if");

        if (state_is_valid)
        {
            if (const core::SpeedControlOut* speedControl =
                    std::get_if<core::SpeedControlOut>(&cmd_out))
            {  // speed controller, set RPM

                desired_rpm_msg->set_drivebrain_set_rpm_fl(
                    speedControl->desired_rpms.FL);
                desired_rpm_msg->set_drivebrain_set_rpm_fr(
                    speedControl->desired_rpms.FR);
                desired_rpm_msg->set_drivebrain_set_rpm_rl(
                    speedControl->desired_rpms.RL);
                desired_rpm_msg->set_drivebrain_set_rpm_rr(
                    speedControl->desired_rpms.RR);

                // same with torque limits
                torque_limit_msg->set_drivebrain_torque_fl(
                    speedControl->torque_lim_nm.FL);
                torque_limit_msg->set_drivebrain_torque_fr(
                    speedControl->torque_lim_nm.FR);
                torque_limit_msg->set_drivebrain_torque_rl(
                    speedControl->torque_lim_nm.RL);
                torque_limit_msg->set_drivebrain_torque_rr(
                    speedControl->torque_lim_nm.RR);

                // spdlog::info(
                //     "CMD_RPM fl={}, fr={}, rl={}, rr={} CMD_TLIM fl={},
                //     fr={}, " "rl={}, rr={}", speedControl->desired_rpms.FL,
                //     speedControl->desired_rpms.FR,
                //     speedControl->desired_rpms.RL,
                //     speedControl->desired_rpms.RR,
                //     speedControl->torque_lim_nm.FL,
                //     speedControl->torque_lim_nm.FR,
                //     speedControl->torque_lim_nm.RL,
                //     speedControl->torque_lim_nm.RR);

                // spdlog::info("tick: send_telem_speed");

                _telem_can->send_message(desired_rpm_msg);
                _telem_can->send_message(torque_limit_msg);

                // // spdlog::info("tick: send_aux_speed");

                _aux_can->send_message(desired_rpm_msg);
                _aux_can->send_message(torque_limit_msg);

                // spdlog::info("tick: log_speed");

                core::log(desired_rpm_msg);
                core::log(torque_limit_msg);
            }
            else if (const core::TorqueControlOut* torqueControl =
                         std::get_if<core::TorqueControlOut>(&cmd_out))
            {  // if it is a torque controller:
                // set desired torque

                desired_torque_msg->set_drivebrain_torque_fl(
                    torqueControl->desired_torques_nm.FL);
                desired_torque_msg->set_drivebrain_torque_fr(
                    torqueControl->desired_torques_nm.FR);
                desired_torque_msg->set_drivebrain_torque_rl(
                    torqueControl->desired_torques_nm.RL);
                desired_torque_msg->set_drivebrain_torque_rr(
                    torqueControl->desired_torques_nm.RR);

                // spdlog::info("tick: send_telem_torque");

                _telem_can->send_message(desired_torque_msg);

                // spdlog::info("tick: send_aux_torque");

                _aux_can->send_message(desired_torque_msg);

                // spdlog::info("tick: log_aux_torque");

                core::log(desired_torque_msg);
            }
        }

        std::tuple<std::string, bool> mcap_status =
            core::MCAPLogger::instance().status();
        std::string logile_name = std::get<0>(mcap_status);

        std::shared_ptr<hytech_msgs::McapInfo> mcap_info =
            std::make_shared<hytech_msgs::McapInfo>();
        mcap_info->set_current_mcap(logile_name);

        core::log(mcap_info);

        auto now = std::chrono::steady_clock::now();
        if (now > next_tick)
        {
            spdlog::warn("Loop overrun by {}", now - next_tick);
            next_tick = now;
        }

        std::this_thread::sleep_until(next_tick);

        // spdlog::info("tick: stop");
    }
}
