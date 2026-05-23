#pragma once

#include "InveterInterface.hpp"
#include <string>

#include <gz/transport13/gz/transport/Node.hh>

class InverterSim : public InverterInterface {
// public:
//   InverterSim(const std::string& topic_prefix = "/inverter") 
//     : rpm_pub_(node_.Advertise<gz::msgs::Float_V>(topic_prefix + "/rpm")),
//       torque_limit_pub_(node_.Advertise<gz::msgs::Float_V>(topic_prefix + "/torque_limit")),
//       torque_pub_(node_.Advertise<gz::msgs::Float_V>(topic_prefix + "/torque")) {}

//   void send_rpm(const hytech::drivebrain_speed_set_input& input) override {
//     gz::msgs::Float_V msg;
//     msg.add_data(input.drivebrain_set_rpm_fl());
//     msg.add_data(input.drivebrain_set_rpm_fr());
//     msg.add_data(input.drivebrain_set_rpm_rl());
//     msg.add_data(input.drivebrain_set_rpm_rr());
//     rpm_pub_.Publish(msg);
//   }

//   void send_torque_limit(const hytech::drivebrain_torque_lim_input& input) override {
//     gz::msgs::Float_V msg;
//     msg.add_data(input.drivebrain_torque_fl());
//     msg.add_data(input.drivebrain_torque_fr());
//     msg.add_data(input.drivebrain_torque_rl());
//     msg.add_data(input.drivebrain_torque_rr());
//     torque_limit_pub_.Publish(msg);
//   }

//   void send_torque(const hytech::drivebrain_desired_torque_input& input) override {
//     gz::msgs::Float_V msg;
//     msg.add_data(input.drivebrain_torque_fl());
//     msg.add_data(input.drivebrain_torque_fr());
//     msg.add_data(input.drivebrain_torque_rl());
//     msg.add_data(input.drivebrain_torque_rr());
//     torque_pub_.Publish(msg);
//     _node->
//   }

// private:
//   gz::transport::Node _node;
//   gz::transport::Node::Publisher rpm_pub_;
//   gz::transport::Node::Publisher torque_limit_pub_;
//   gz::transport::Node::Publisher torque_pub_;
};
