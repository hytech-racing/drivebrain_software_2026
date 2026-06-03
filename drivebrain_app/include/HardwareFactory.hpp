#pragma once

#include "InveterInterface.hpp"
#include "LidarInterface.hpp"
#include <memory>
#include <string>

#ifdef HOOTL_ENABLED
#include "InverterSim.hpp"
#include "SimLidar.hpp"
#include "SimStateReceiver.hpp"

inline std::unique_ptr<InverterInterface> make_inverter(const std::string& /*dbc_path*/) {
    return std::make_unique<InverterSim>();
}

inline std::unique_ptr<LidarInterface> make_lidar() {
    return std::make_unique<SimLidar>();
}

inline std::unique_ptr<SimStateReceiver> make_sim_state_receiver() {
    auto r = std::make_unique<SimStateReceiver>();
    r->start();
    return r;
}

#else
#include "CANInverter.hpp"
#include "Lidar.hpp"
#include "FoxgloveServer.hpp"
#include "CANComms.hpp"

inline std::unique_ptr<InverterInterface> make_inverter(const std::string& dbc_path) {
    auto telem_can = std::make_unique<comms::CANComms>(
        core::FoxgloveServer::instance().get_param<std::string>("telem_can_device").value(), dbc_path);
    auto aux_can = std::make_unique<comms::CANComms>(
        core::FoxgloveServer::instance().get_param<std::string>("aux_can_device").value(), dbc_path);
    return std::make_unique<CANInverter>(std::move(telem_can), std::move(aux_can));
}

inline std::unique_ptr<LidarInterface> make_lidar() {
    return std::make_unique<Lidar>();
}
#endif
