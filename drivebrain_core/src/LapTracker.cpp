#include <LapTracker.hpp>
#include <cmath>

namespace {

double line_side(double line_ax, double line_ay, double line_bx, double line_by,
                 double px, double py) {
    double line_dx = line_bx - line_ax;
    double line_dy = line_by - line_ay;
    double pt_dx = px - line_ax;
    double pt_dy = py - line_ay;
    return pt_dx * line_dy - pt_dy * line_dx;
}

}  // namespace

void core::LapTracker::step_tracker(core::VehicleState& latest_state) {
    std::shared_ptr<hytech_msgs::LapTime> laptime_information = std::make_shared<hytech_msgs::LapTime>();

    float current_speed_mps = std::sqrt(
        latest_state.current_body_vel_ms.x * latest_state.current_body_vel_ms.x +
        latest_state.current_body_vel_ms.y * latest_state.current_body_vel_ms.y +
        latest_state.current_body_vel_ms.z * latest_state.current_body_vel_ms.z);
    _max_lap_speed_mps = std::max(_max_lap_speed_mps, current_speed_mps);

    bool line_configured = (_line_start_lat != _line_end_lat) || (_line_start_lon != _line_end_lon);
    if (latest_state.vehicle_position.valid && line_configured) {
        double curr_lat = latest_state.vehicle_position.lat;
        double curr_lon = latest_state.vehicle_position.lon;
        double curr_side = line_side(_line_start_lat, _line_start_lon,
                                    _line_end_lat, _line_end_lon,
                                    curr_lat, curr_lon);

        if (_has_prev_position) {
            if ((_prev_side > 0 && curr_side < 0) || (_prev_side < 0 && curr_side > 0)) {
                _lap_count++;
                _lap_start_time = std::chrono::steady_clock::now();
                _max_lap_speed_mps = 0.f;
            }
        } else {
            _has_prev_position = true;
        }
        _prev_side = curr_side;
    }

    auto now = std::chrono::steady_clock::now();
    float laptime_seconds = std::chrono::duration<float>(now - _lap_start_time).count();

    laptime_information->set_laptime_seconds(laptime_seconds);
    laptime_information->set_lapcount(static_cast<int64_t>(_lap_count));
    laptime_information->set_max_lap_speed_ms(_max_lap_speed_mps);

    core::StateTracker::instance().handle_receive_protobuf_message(laptime_information);
}

void core::LapTracker::create() {
    LapTracker* expected = nullptr;
    LapTracker* local = new LapTracker();
    if(!_s_instance.compare_exchange_strong(expected, local, std::memory_order_release, std::memory_order_relaxed)) {
        // Already initialized, delete local instance
        delete local;
    }
}

core::LapTracker::LapTracker()
    : _lap_start_time(std::chrono::steady_clock::now()) {}

void core::LapTracker::set_start_finish_line(double start_lat, double start_lon,
                                             double end_lat, double end_lon) {
    _line_start_lat = start_lat;
    _line_start_lon = start_lon;
    _line_end_lat = end_lat;
    _line_end_lon = end_lon;
}

core::LapTracker& core::LapTracker::instance() {
    LapTracker* instance = _s_instance.load(std::memory_order_acquire);
    assert(instance != nullptr && "LapTracker has not been initialized");
    return *instance;
}

