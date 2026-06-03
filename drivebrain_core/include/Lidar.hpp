#pragma once

#include "LidarInterface.hpp"
#include <spdlog/spdlog.h>

class Lidar : public LidarInterface {
public:
    void start() override {
        spdlog::warn("check back when ouster gives us a lidar or something");
    }
};
