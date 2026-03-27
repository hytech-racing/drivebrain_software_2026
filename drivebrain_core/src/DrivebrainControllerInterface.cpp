#include <DrivebrainControllerInterface.hpp>

/** Singleton Methods */
void core::DrivebrainControllerInterface::create() {
    DrivebrainControllerInterface* expected = nullptr;
    DrivebrainControllerInterface* local = new DrivebrainControllerInterface();
    if(!_s_instance.compare_exchange_strong(expected, local, std::memory_order_release, std::memory_order_relaxed)) {
        // Already initialized, delete local instance
        delete local;
    }
}

core::DrivebrainControllerInterface& core::DrivebrainControllerInterface::instance() {
    DrivebrainControllerInterface* instance = _s_instance.load(std::memory_order_acquire);
    assert(instance != nullptr && "Drivebrain Controller Interface has not been initialized");
    return *instance;
}

void core::DrivebrainControllerInterface::destroy() {
    DrivebrainControllerInterface* instance = _s_instance.exchange(nullptr, std::memory_order_acq_rel);
    if (instance) {
        delete instance;
    }
}

core::DrivebrainControllerInterface::DrivebrainControllerInterface() {
    _response = "NONE";
}

/* Private Methods */
void core::DrivebrainControllerInterface::_handle_parameter_updates(const std::unordered_map<std::string, foxglove::ParameterValue> &new_params) {
    
    if (new_params.find("should_log") != new_params.end()) {
        bool should_log = new_params.at("should_log").getValue<bool>();
        if (should_log) {
            _request_start_logging();
        } else {
            _request_stop_logging();
        }
    }

    if (new_params.find("controller_index") != new_params.end()) {
        int controller_index = new_params.at("controller_index").getValue<int>();
        _request_controller_change(controller_index);
    }

}

void core::DrivebrainControllerInterface::_request_start_logging() {
    std::tuple<std::string, bool> mcap_status = core::MCAPLogger::instance().status();
    bool is_logging = std::get<1>(mcap_status);
    if (!is_logging) {
        // Start logging
    }
}

void core::DrivebrainControllerInterface::_request_stop_logging() {
    
}





