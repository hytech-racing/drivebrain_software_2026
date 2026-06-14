#pragma once

#include "SteeringInterface.hpp"

#include <atomic>
#include <cmath>

class SteeringSim : public SteeringInterface {
public:
    void send_steering(const hytech_msgs::DBSteeringCommand& cmd) override {
        _steer_rad.store(
            static_cast<double>(cmd.desired_steering_angle_deg()) * (M_PI / 180.0),
            std::memory_order_relaxed);
    }

    static double commanded_steer_rad() {
        return _steer_rad.load(std::memory_order_relaxed);
    }

private:
    static inline std::atomic<double> _steer_rad{0.0};
};
