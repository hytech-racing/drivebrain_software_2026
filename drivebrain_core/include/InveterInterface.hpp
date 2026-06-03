#pragma once
#include "hytech.pb.h"

/**
 * @brief A generic interface for commanding inverters.
 *
 * Different hardware/simulation interfaces can implement this interface to allow for easy swappage in the main app.
*/
class InverterInterface {
public:
  virtual ~InverterInterface() = default;
  virtual void send_rpm(const hytech::drivebrain_speed_set_input&) = 0;
  virtual void send_torque_limit(const hytech::drivebrain_torque_lim_input&) = 0;
  virtual void send_torque(const hytech::drivebrain_desired_torque_input&) = 0;

  static std::unique_ptr<InverterInterface> create(const std::string& dbc_path);
};
