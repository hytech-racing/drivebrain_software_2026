#ifndef DRIVEBRAIN_APP_H
#define DRIVEBRAIN_APP_H

#pragma once

#include "SurreyAeroComms.hpp"

#include <SpeedTechComms.hpp>
#include <ScaleComms.hpp>
#include <JsonFileHandler.hpp>
#include <CANComms.hpp>

#include <SimpleSpeedController.hpp>
#include <LoadCellVectoringTorqueController.hpp>

#include <ControllerManager.hpp>
#include <VNComms.hpp>
#include <MsgLogger.hpp>
#include <DrivebrainMCAPLogger.hpp>
#include <hytech_msgs.pb.h>
#include <mcap/writer.hpp>
#include <DrivebrainBase.hpp>
#include <foxglove_server.hpp>
#include <DBServiceImpl.hpp>
#include <ETHRecvComms.hpp>

#include <EstimatorManager.hpp>

#include <thread>
#include <chrono>
#include <condition_variable>
#include <cassert>
#include <boost/program_options.hpp>
#include <boost/asio.hpp>
#include <memory>
#include <optional>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <spdlog/spdlog.h>
#include <atomic>

struct DriveBrainSettings {

    bool run_db_service{true};
    bool run_io_context{true};
    bool run_process_loop{true};
    bool use_vectornav{true};
    bool use_secondary_can{true};

};

class DriveBrainApp {

public:
    DriveBrainApp(const std::string& config_path, const std::string& dbc_path, const DriveBrainSettings& settings = DriveBrainSettings{});
    ~DriveBrainApp();

    void run();
    void stop();

private:

void _process_loop();
    void _signal_handler(int signal);
    void _setup_loggers(std::vector<std::shared_ptr<core::common::Loggable<std::shared_ptr<google::protobuf::Message>>>> logging_components);

private:

    static std::atomic<bool> _stop_signal;
    const std::string _param_path;
    core::Logger _logger;
    core::JsonFileHandler _config;
    std::optional<std::string> _dbc_path;
    boost::asio::io_context _io_context;
    boost::asio::io_context _io_context_secondary_can;
    // boost::asio::io_context _aero_usb_io_context;
    // boost::asio::io_context _io_context_speed_tech_serial;
    // boost::asio::io_context _scale_usb_io_context;
    
    // core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> _rx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> _primary_can_tx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> _secondary_can_tx_queue;
    // core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> _eth_tx_queue;
    // core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> _live_telem_queue;

    // std::vector<std::shared_ptr<core::common::Configurable>> _configurable_components;
    // std::shared_ptr<common::DrivebrainMCAPLogger> _mcap_logger;
    // std::shared_ptr<control::SimpleSpeedController> controller1;
    // std::shared_ptr<control::LoadCellVectoringTorqueController> _mode1;


    // control::ControllerManager<control::Controller<core::ControllerOutput, core::VehicleState>, 2 + matlab_model_gen::num_controllers> _controllerManager;

    // std::shared_ptr<core::FoxgloveWSServer> _foxglove_server;
    // std::shared_ptr<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>> _message_logger = nullptr;
    // std::shared_ptr<core::StateEstimator> _state_estimator;
    std::shared_ptr<comms::CANDriver> _driver_primary_can;
    std::shared_ptr<comms::CANDriver> _driver_secondary_can;
    // std::shared_ptr<comms::SurreyAeroComms> _aero_sensor_driver = nullptr;
    // std::shared_ptr<comms::SpeedTechComms> _lap_timer_driver;
    // bool _using_lap_timer = false;
    // std::shared_ptr<comms::ETHRecvComms<hytech_msgs::ACUAllData>> _acu_eth_driver;
    // std::shared_ptr<comms::ETHRecvComms<hytech_msgs::VCRData_s>> _vcr_eth_driver;
    // std::shared_ptr<comms::ETHRecvComms<hytech_msgs::VCFData_s>> _vcf_eth_driver;
    // std::shared_ptr<comms::ETHRecvComms<hytech_msgs::VNData>> _fake_vn = nullptr;
    // std::shared_ptr<comms::VNDriver> _vn_driver;
    // std::unique_ptr<DBInterfaceImpl> _db_service;
    
    // std::shared_ptr<comms::ScaleComms> _scale_comms = nullptr;

    // std::vector<std::shared_ptr<MatlabModel>> _gend_controllers;

    // std::shared_ptr<estimation::EstimatorManager> _estim_manager;
    // std::thread _process_thread;
    std::thread _io_context_thread;
    std::thread _io_context_secondary_thread;
    // std::thread _db_service_thread;
    // std::thread _aero_usb_io_context_thread;
    // std::thread _io_context_speed_tech_serial_thread;
    // std::thread _scale_usb_io_context_thread;
    const DriveBrainSettings _settings;

    std::chrono::steady_clock::time_point _last_send_time = std::chrono::steady_clock::now();
    const std::chrono::seconds _send_period{1}; // 1 Hz

};

#endif // DRIVEBRAIN_APP_H