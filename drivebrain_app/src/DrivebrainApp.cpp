#include "DrivebrainApp.hpp"

std::atomic<bool> stop_signal{false};

DrivebrainApp::DriveBrainApp(const std::string& config_path, const std::string& dbc_path, const DriveBrainSettings& settings = DriveBrainSettings{}) :
    _param_path(param_path),
    _dbc_path(dbc_path),
    _config(config_path),
    _settings(settings)
     {

    _driver_primary_can = std::make_shared<comms::CANDriver>(
        _config, _primary_can_tx_queue, _io_context, 
        _dbc_path, construction_failed, _state_estimator, "CANDriverPrimary"
    );

}

void DrivebrainApp::run() {

      _io_context_thread = std::thread([this]() {
        if (!_settings.run_io_context) return;
        spdlog::info("Started io context thread");
        try {
            _io_context.run();
        } catch (const std::exception& e) {
            spdlog::error("Error in io_context: {}", e.what());
        }
    });

}