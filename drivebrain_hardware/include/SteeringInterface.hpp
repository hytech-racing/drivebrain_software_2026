#pragma once
#include "hytech_msgs.pb.h"
#include <memory>

class SteeringInterface {
public:
    virtual ~SteeringInterface() = default;
    virtual void send_steering(const hytech_msgs::DBSteeringCommand&) = 0;

    static std::unique_ptr<SteeringInterface> create();
};
