#ifdef HOOTL_ENABLED
#include "InverterSim.hpp"
#else
#include "CANInverter.hpp"
#include "CANComms.hpp"
#include "FoxgloveServer.hpp"
#endif

std::unique_ptr<InverterInterface> InverterInterface::create(const std::string& dbc_path) {
#ifdef HOOTL_ENABLED
  (void) dbc_path;
  return std::make_unique<InverterSim>();
#else
  auto telem_can = std::make_unique<comms::CANComms>(core::FoxgloveServer::instance().get_param<std::string>("telem_can_device").value(), dbc_path);
  auto aux_can = std::make_unique<comms::CANComms>(core::FoxgloveServer::instance().get_param<std::string>("aux_can_device").value(), dbc_path);

  return std::make_unique<CANInverter>(std::move(telem_can), std::move(aux_can));
#endif
}
