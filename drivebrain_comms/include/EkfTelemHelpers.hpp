#pragma once

#include <google/protobuf/any.pb.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/message.h>

#include <memory>

#include "htx_ekf/ekf_manager.hpp"
#include "hytech_msgs.pb.h"

namespace comms
{

std::shared_ptr<hytech_msgs::EkfState> make_ekf_state_msg(
    const htx_ekf::EkfOutput& state);

std::shared_ptr<hytech_msgs::EkfDebugSnapshot> make_ekf_debug_snapshot_msg(
    const htx_ekf::EkfDebugSnapshot& debug);

void publish_ekf_step_result(const htx_ekf::EkfStepResult& result);

}  // namespace comms