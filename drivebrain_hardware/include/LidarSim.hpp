#pragma once

#include "LidarInterface.hpp"
#include "LidarOdometry.hpp"
#include "Telemetry.hpp"
#include "foxglove/PointCloud.pb.h"
#include "foxglove/PackedElementField.pb.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>
#include <zmq.hpp>

struct LidarScanHeader {
    int64_t  timestamp_ns;
    uint32_t num_points;
};

class LidarSim : public LidarInterface {
public:
    LidarSim()
        : _ctx(1), _pull(_ctx, zmq::socket_type::pull)
    {
        _pull.set(zmq::sockopt::rcvtimeo, 100);
        _pull.connect("ipc:///tmp/hytech_sim_lidar");
    }

    ~LidarSim() {
        _running = false;
        if (_thread.joinable()) _thread.join();
    }

    void start() override {
        _running = true;
        _thread = std::thread([this]() { _recv_loop(); });
    }

private:
    void _recv_loop() {
        while (_running) {
            zmq::message_t msg;
            if (!_pull.recv(msg, zmq::recv_flags::none)) continue;
            if (msg.size() < sizeof(LidarScanHeader)) continue;

            const auto* hdr = msg.data<LidarScanHeader>();
            if (msg.size() != sizeof(LidarScanHeader) + hdr->num_points * 12u) continue;

            auto pc = std::make_shared<foxglove::PointCloud>();
            pc->mutable_timestamp()->set_seconds(hdr->timestamp_ns / 1'000'000'000LL);
            pc->mutable_timestamp()->set_nanos(hdr->timestamp_ns % 1'000'000'000LL);
            pc->set_frame_id("lidar");
            /* Identity pose — cloud sits at the origin of the "lidar" frame. Foxglove's
               3D panel won't render a cloud with a null pose. */
            pc->mutable_pose()->mutable_position()->set_x(0.0);
            pc->mutable_pose()->mutable_position()->set_y(0.0);
            pc->mutable_pose()->mutable_position()->set_z(0.0);
            pc->mutable_pose()->mutable_orientation()->set_w(1.0);
            pc->set_point_stride(12);

            for (auto [name, off] : {std::pair{"x", 0u}, {"y", 4u}, {"z", 8u}}) {
                auto* f = pc->add_fields();
                f->set_name(name);
                f->set_offset(off);
                f->set_type(foxglove::PackedElementField::FLOAT32);
            }

            pc->set_data(reinterpret_cast<const char*>(hdr + 1), hdr->num_points * 12u);

            static std::size_t rx = 0;
            if (rx++ % 100 == 0)
                spdlog::info("[LidarSim] recv #{} points={} bytes={}", rx, hdr->num_points, msg.size());

            core::log(pc);

            _odometry.update(reinterpret_cast<const float*>(hdr + 1),
                             hdr->num_points, hdr->timestamp_ns);
        }
    }

    zmq::context_t    _ctx;
    zmq::socket_t     _pull;
    std::thread       _thread;
    std::atomic<bool> _running{false};
    LidarOdometry     _odometry;
};
