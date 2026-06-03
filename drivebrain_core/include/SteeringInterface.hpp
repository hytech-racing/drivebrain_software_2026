#pragma once
#include "hytech_msgs.pb.h"

class SteeringInterface {
public:
    virtual ~SteeringInterface() = default;
    virtual void send_steering(const hytech_msgs::DrivebrainSteeringCommand&) = 0;
};
