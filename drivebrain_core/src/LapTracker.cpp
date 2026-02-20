#include <LapTracker.hpp> 

void core::LapTracker::step_tracker(core::VehicleState& latest_state) {
    std::shared_ptr<hytech_msgs::LapTime> laptime_information = std::make_shared<hytech_msgs::LapTime>();

    auto now = std::chrono::steady_clock::now();
    float time_differential = std::chrono::duration<float>(now - _last_timestamp).count();
    _last_timestamp = now;

    // Logic if car has NOT started racing yet
    if (!_started) {

        if (latest_state.is_ready_to_drive) {

            bool car_stationary = (std::abs(latest_state.current_rpms.FL) < STATIONARY_WHEEL_ERROR
                                && std::abs(latest_state.current_rpms.FR) < STATIONARY_WHEEL_ERROR
                                && std::abs(latest_state.current_rpms.RL) < STATIONARY_WHEEL_ERROR
                                && std::abs(latest_state.current_rpms.RR) < STATIONARY_WHEEL_ERROR)
                                && (std::sqrt(std::pow(latest_state.current_body_vel_ms.x, 2) + std::pow(latest_state.current_body_vel_ms.y, 2)) < MINIMUM_CAR_SPEED);
            bool car_racing = (std::abs(latest_state.current_rpms.FL) > MINIMUM_WHEEL_ROTATION
                            && std::abs(latest_state.current_rpms.FR) > MINIMUM_WHEEL_ROTATION
                            && std::abs(latest_state.current_rpms.RL) > MINIMUM_WHEEL_ROTATION
                            && std::abs(latest_state.current_rpms.RR) > MINIMUM_WHEEL_ROTATION)
                            && (std::sqrt(std::pow(latest_state.current_body_vel_ms.x, 2) + std::pow(latest_state.current_body_vel_ms.y, 2)) > MINIMUM_CAR_SPEED);
            
            // If the car is stationary, assume it is in the start box and set the start line latitutde and longitude position
            //      Elif the car is moving, assume it has started racing
            if (car_stationary) {
                _start_lat = latest_state.vehicle_position.lat;
                _start_lon = latest_state.vehicle_position.lon;
            } else if (car_racing) {
                _started = true;
            }
        }

    // Logic if the car has already started racing
    } else {

        float current_speed = std::sqrt(std::pow(latest_state.current_body_vel_ms.x, 2) + std::pow(latest_state.current_body_vel_ms.y, 2));

        bool crossed_start_lattitude = (_previous_state.vehicle_position.lat <= _start_lat && latest_state.vehicle_position.lat >= _start_lat) 
                                   || (latest_state.vehicle_position.lat <= _start_lat && _previous_state.vehicle_position.lat >= _start_lat);
        bool crossed_start_longitude = (_previous_state.vehicle_position.lon <= _start_lon && latest_state.vehicle_position.lon >= _start_lon)
                                    || (latest_state.vehicle_position.lon <= _start_lon && _previous_state.vehicle_position.lon >= _start_lon);
        bool within_start_lattitude_tolerance = std::abs(latest_state.vehicle_position.lat - _start_lat) <= FINISH_LINE_POSITION_TOLERANCE;
        bool within_start_longitude_tolerance = std::abs(latest_state.vehicle_position.lon - _start_lon) <= FINISH_LINE_POSITION_TOLERANCE;

        // If the car has started has passed the start box, assume the car has completed a lap and update/reset lap metrics accordingly
        //      Else the car has started but is not at the start line, so update mid-lap metrics normally
        if ((_laptime > 10.0f) && ((crossed_start_lattitude && crossed_start_longitude) || (crossed_start_lattitude && within_start_longitude_tolerance) || (crossed_start_longitude && within_start_lattitude_tolerance))) {

            if (_best_laptime == 0.0f || _laptime < _best_laptime) {
                _best_laptime = _laptime;
            }
            
            _last_laptime = _laptime;

            _delta = _best_laptime - _laptime;
            _max_lap_speed = _min_lap_speed = current_speed;
            _lapcount++;
            _laptime = 0.0f;

        } else {
            _max_lap_speed = std::max(current_speed, _max_lap_speed);
            _min_lap_speed = std::min(current_speed, _min_lap_speed);
            _laptime += time_differential;
        }
    }

    _previous_state = latest_state;

    laptime_information->set_laptime_seconds(_laptime);
    laptime_information->set_live_delta(_delta); // TODO: Make this live delta instead of just delta against best lap
    laptime_information->set_best_laptime_seconds(_best_laptime);
    laptime_information->set_last_laptime_seconds(_last_laptime);
    laptime_information->set_lapcount(_lapcount);
    laptime_information->set_max_lap_speed_ms(_max_lap_speed);
    laptime_information->set_min_lap_speed_ms(_min_lap_speed);

    core::StateTracker::instance().handle_receive_protobuf_message(laptime_information); // What "records" the information
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







