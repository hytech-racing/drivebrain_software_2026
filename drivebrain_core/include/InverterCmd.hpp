#pragma once

#include "hytech.pb.h"

class InverterCmd {
  
  virtual void set_rpm_req() = 0;
  virtual void set_torque_lim_req() = 0;
  virtual void set_torque_req() = 0;
};
