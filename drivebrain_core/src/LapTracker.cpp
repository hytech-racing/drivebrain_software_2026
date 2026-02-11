#include <LapTracker.hpp> 
#include <cassert>
#include <cmath>

namespace {
    constexpr double DEG_TO_RAD = 0.017453292519943295;
    constexpr double METERS_PER_DEG_LAT = 111320.0;
    constexpr float START_SPEED_THRESHOLD_MS = 2.0f;
    constexpr float STATIONARY_SPEED_THRESHOLD_MS = 0.75f;
    constexpr double START_ZONE_RADIUS_M = 7.0;
    constexpr float MIN_LAP_SECONDS = 20.0f;

    double distance_meters(double lat_a, double lon_a, double lat_b, double lon_b) {
        const double avg_lat_rad = ((lat_a + lat_b) * 0.5) * DEG_TO_RAD;
        const double d_lat_m = (lat_a - lat_b) * METERS_PER_DEG_LAT;
        const double d_lon_m = (lon_a - lon_b) * METERS_PER_DEG_LAT * std::cos(avg_lat_rad);
        return std::sqrt((d_lat_m * d_lat_m) + (d_lon_m * d_lon_m));
    }
}

void core::LapTracker::step_tracker(core::VehicleState& latest_state) {
    std::shared_ptr<hytech_msgs::LapTime> laptime_information = std::make_shared<hytech_msgs::LapTime>(); // TODO you need to fill in the fields of this protobuf message
    /**
     * TODO it is your responsibility to fill in this method. Look at the VehicleState 
     * struct to see all the things it gives you (you should see vn position from there). Using this
     * and the private variables you added in the header, complete this method. It should use all of the information
     * in the latest state to update it's local variables, create a LapTime protobuf, and invoke handle_receive_protobuf_message
     * on the state tracker. Some of this is completed for you. Good luck!
     */
    const auto now = std::chrono::steady_clock::now();

    const float body_speed_ms = std::sqrt(
        (latest_state.current_body_vel_ms.x * latest_state.current_body_vel_ms.x) +
        (latest_state.current_body_vel_ms.y * latest_state.current_body_vel_ms.y));

    const bool has_valid_position = latest_state.vehicle_position.valid &&
                                    std::isfinite(latest_state.vehicle_position.lat) &&
                                    std::isfinite(latest_state.vehicle_position.lon);

    if (has_valid_position && !_lap_started && body_speed_ms < STATIONARY_SPEED_THRESHOLD_MS) {
        _start_lat = latest_state.vehicle_position.lat;
        _start_lon = latest_state.vehicle_position.lon;
        _has_start_reference = true;
    }

    if (_has_start_reference && !_lap_started && body_speed_ms > START_SPEED_THRESHOLD_MS) {
        _lap_started = true;
        _was_outside_start_zone = false;
        _lap_start_timestamp = now;
        _laptime_seconds = 0.0f;
    }

    if (_lap_started) {
        _laptime_seconds = std::chrono::duration<float>(now - _lap_start_timestamp).count();
    }

    if (_lap_started && _has_start_reference && has_valid_position) {
        const double distance_to_start_m = distance_meters(
            latest_state.vehicle_position.lat,
            latest_state.vehicle_position.lon,
            _start_lat,
            _start_lon);
        const bool is_in_start_zone = distance_to_start_m <= START_ZONE_RADIUS_M;
        if (!is_in_start_zone) {
            _was_outside_start_zone = true;
        } else if (_was_outside_start_zone && _laptime_seconds >= MIN_LAP_SECONDS) {
            _lapcount++;
            _lap_start_timestamp = now;
            _laptime_seconds = 0.0f;
            _was_outside_start_zone = false;
        }
    }

    laptime_information->set_laptime_seconds(_laptime_seconds);
    laptime_information->set_lapcount(_lapcount);
    core::StateTracker::instance().handle_receive_protobuf_message(laptime_information); // What "records" the information

}

core::LapTracker::LapTracker() {
    _lap_start_timestamp = std::chrono::steady_clock::now();
}

void core::LapTracker::create() {
    LapTracker* expected = nullptr;
    LapTracker* local = new LapTracker();
    if(!_s_instance.compare_exchange_strong(expected, local, std::memory_order_release, std::memory_order_relaxed)) {
        // Already initialized, delete local instance
        delete local;
    }
}

core::LapTracker& core::LapTracker::instance() {
    LapTracker* instance = _s_instance.load(std::memory_order_acquire);
    assert(instance != nullptr && "LapTracker has not been initialized");
    return *instance;
}

