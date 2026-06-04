#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <zmq.hpp>
#include "Telemetry.hpp"
#include "StateTracker.hpp"
#include "sim_msgs.pb.h"
#include "foxglove/FrameTransform.pb.h"
#include "hytech.pb.h"

static constexpr double LIDAR_MOUNT_X = 0.75;
static constexpr double LIDAR_MOUNT_Z = 0.15;

struct SimState {
    double ax_world, ay_world, psi_ddot_world;
    double ax_body, ay_body, psi_ddot_body;
    double vx_world, vy_world, psi_dot_world;
    double vx_body, vy_body, psi_dot_body;
    double x_world, y_world, psi_world;
};

class SimStateReceiver {
public:
    SimStateReceiver()
        : _ctx(1), _pull(_ctx, zmq::socket_type::pull)
    {
        _pull.set(zmq::sockopt::rcvtimeo, 100);
        _pull.connect("ipc:///tmp/hytech_sim_state");
    }

    ~SimStateReceiver() {
        _running = false;
        if (_thread.joinable()) _thread.join();
    }

    void start() {
        _running = true;
        _thread = std::thread([this]() { _recv_loop(); });
    }

private:
    void _recv_loop() {
        while (_running) {
            zmq::message_t msg;
            auto res = _pull.recv(msg, zmq::recv_flags::none);
            if (!res) continue;
            if (msg.size() != sizeof(SimState)) {
                static int bad_dbg = 0;
                if (bad_dbg++ % 100 == 0)
                    spdlog::warn("[DBG SimStateRecv] size mismatch: got {} expected {}", msg.size(), sizeof(SimState));
                continue;
            }

            SimState s{};
            std::memcpy(&s, msg.data(), sizeof(s));

            // TEMP DEBUG
            static int sim_dbg = 0;
            if (sim_dbg++ % 250 == 0)
                spdlog::info("[DBG SimStateRecv] got state #{} x={:.2f} y={:.2f}", sim_dbg, s.x_world, s.y_world);

            auto gt = std::make_shared<sim_msgs::SimGroundTruth>();
            gt->set_x_world(s.x_world);
            gt->set_y_world(s.y_world);
            gt->set_psi_world(s.psi_world);
            gt->set_vx_body(s.vx_body);
            gt->set_vy_body(s.vy_body);
            gt->set_psi_dot_body(s.psi_dot_body);
            gt->set_ax_body(s.ax_body);
            gt->set_ay_body(s.ay_body);

            core::log(gt);

            auto tf = std::make_shared<foxglove::FrameTransform>();
            tf->set_parent_frame_id("odom");
            tf->set_child_frame_id("lidar");
            tf->mutable_translation()->set_x(s.x_world + LIDAR_MOUNT_X * std::cos(s.psi_world));
            tf->mutable_translation()->set_y(s.y_world + LIDAR_MOUNT_X * std::sin(s.psi_world));
            tf->mutable_translation()->set_z(LIDAR_MOUNT_Z);
            tf->mutable_rotation()->set_z(std::sin(s.psi_world * 0.5));
            tf->mutable_rotation()->set_w(std::cos(s.psi_world * 0.5));
            {
                auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
                tf->mutable_timestamp()->set_seconds(ns / 1'000'000'000LL);
                tf->mutable_timestamp()->set_nanos(static_cast<int32_t>(ns % 1'000'000'000LL));
            }
            core::log(tf);

            // Feed synthetic low-level messages so StateTracker timestamps stay fresh
            auto rear_sus = std::make_shared<hytech::rear_suspension>();
            auto front_sus = std::make_shared<hytech::front_suspension>();
            auto pedals = std::make_shared<hytech::pedals_system_data>();
            auto steering = std::make_shared<hytech::steering_data>();

            core::StateTracker::instance().handle_receive_protobuf_message(rear_sus);
            core::StateTracker::instance().handle_receive_protobuf_message(front_sus);
            core::StateTracker::instance().handle_receive_protobuf_message(pedals);
            core::StateTracker::instance().handle_receive_protobuf_message(steering);
        }
    }

    zmq::context_t _ctx;
    zmq::socket_t _pull;
    std::thread _thread;
    std::atomic<bool> _running{false};
};
