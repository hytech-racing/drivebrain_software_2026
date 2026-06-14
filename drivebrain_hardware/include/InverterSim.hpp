#pragma once

#include "InveterInterface.hpp"
#include "SteeringSim.hpp"

#include <zmq.hpp>
#include <cmath>
#include <cstring>

struct SimCmd {
    double torque_fl;
    double torque_fr;
    double torque_rl;
    double torque_rr;
    double wheel_steer_rad;
};

class InverterSim : public InverterInterface {
public:
    InverterSim()
        : _ctx(1)
        , _cmd_push(_ctx, zmq::socket_type::push)
    {
        _cmd_push.connect("ipc:///tmp/hytech_sim_cmd");
    }

    /* RPM and torque-limit have no meaning in fmu sim */
    void send_rpm(const hytech::drivebrain_speed_set_input&) override {}
    void send_torque_limit(const hytech::drivebrain_torque_lim_input&) override {}
    void send_torque(const hytech::drivebrain_desired_torque_input& input) override {
        SimCmd cmd{
            .torque_fl = input.drivebrain_torque_fl(),
            .torque_fr = input.drivebrain_torque_fr(),
            .torque_rl = input.drivebrain_torque_rl(),
            .torque_rr = input.drivebrain_torque_rr(),
            .wheel_steer_rad = SteeringSim::commanded_steer_rad()
        };
        zmq::message_t msg(sizeof(cmd));
        std::memcpy(msg.data(), &cmd, sizeof(cmd));
        _cmd_push.send(msg, zmq::send_flags::dontwait);
    }

private:
    zmq::context_t _ctx;
    zmq::socket_t _cmd_push;
};
