#pragma once
#include <memory>

class LidarInterface {
public:
    virtual ~LidarInterface() = default;
    virtual void start() = 0;

    static std::unique_ptr<LidarInterface> create();
};
