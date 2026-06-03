#pragma once

#include "InveterInterface.hpp"
#include "CANComms.hpp"
#include <memory>

class CANInverter : public InverterInterface {
public:
    CANInverter(std::unique_ptr<comms::CANComms> telem_can,
                std::unique_ptr<comms::CANComms> aux_can)
        : _telem_can(std::move(telem_can))
        , _aux_can(std::move(aux_can))
    {}

    void send_rpm(const hytech::drivebrain_speed_set_input& msg) override {
        auto shared = std::make_shared<hytech::drivebrain_speed_set_input>(msg);
        _telem_can->send_message(shared);
        _aux_can->send_message(shared);
    }

    void send_torque_limit(const hytech::drivebrain_torque_lim_input& msg) override {
        auto shared = std::make_shared<hytech::drivebrain_torque_lim_input>(msg);
        _telem_can->send_message(shared);
        _aux_can->send_message(shared);
    }

    void send_torque(const hytech::drivebrain_desired_torque_input& msg) override {
        auto shared = std::make_shared<hytech::drivebrain_desired_torque_input>(msg);
        _telem_can->send_message(shared);
        _aux_can->send_message(shared);
    }

private:
    std::unique_ptr<comms::CANComms> _telem_can;
    std::unique_ptr<comms::CANComms> _aux_can;
};
