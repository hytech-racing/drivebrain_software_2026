#pragma once

#include <memory>
#include <cstring>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Telemetry.hpp"
#include "foxglove/FrameTransform.pb.h"
#include "foxglove/PointCloud.pb.h"
#include "foxglove/PackedElementField.pb.h"

#include <spdlog/spdlog.h>

class LidarOdometry {
public:
    LidarOdometry() = default;

    // TODO: reimplement with factor graph
    void update(const float* xyz, uint32_t n_pts, int64_t timestamp_ns) {
        (void)xyz;
        (void)n_pts;
        (void)timestamp_ns;
    }

private:
    Eigen::Isometry3d _pose = Eigen::Isometry3d::Identity();
};
