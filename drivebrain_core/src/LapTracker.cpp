#include <LapTracker.hpp> 

void core::LapTracker::step_tracker(core::VehicleState& latest_state) {
    std::shared_ptr<hytech_msgs::LapTime> laptime_information = std::make_shared<hytech_msgs::LapTime>(); // TODO you need to fill in the fields of this protobuf message
    /**
     * TODO it is your responsibility to fill in this method. Look at the VehicleState 
     * struct to see all the things it gives you (you should see vn position from there). Using this
     * and the private variables you added in the header, complete this method. It should use all of the information
     * in the latest state to update it's local variables, create a LapTime protobuf, and invoke handle_receive_protobuf_message
     * on the state tracker. Some of this is completed for you. Good luck!
     */
    
     // Assuming 1 rpms correponds to 1 real life RPM


    auto now = std::chrono::steady_clock::now();
    float time_differential = std::chrono::duration<float>(now - _last_timestamp).count();
    _last_timestamp = now;

    if (_lapcount == 0
        && latest_state.is_ready_to_drive
        && std::abs(latest_state.current_rpms.FL) < _error
        && std::abs(latest_state.current_rpms.FR) < _error
        && std::abs(latest_state.current_rpms.RL) < _error
        && std::abs(latest_state.current_rpms.RR) < _error
        && std::abs(latest_state.current_body_vel_ms.x) < _error
        && std::abs(latest_state.current_body_vel_ms.y) < _error)
    {
        // Assume car is in start box
        _start_lat = latest_state.vehicle_position.lat;
        _start_lon = latest_state.vehicle_position.lon;
    }
    else if (!_started
        && std::abs(latest_state.current_rpms.FL) > 10
        && std::abs(latest_state.current_rpms.FR) > 10
        && std::abs(latest_state.current_rpms.RL) > 10
        && std::abs(latest_state.current_rpms.RR) > 10
        && (std::abs(latest_state.current_body_vel_ms.x) > 0.2
        || std::abs(latest_state.current_body_vel_ms.y) > 0.2))
    {
        // Assume car has started
        _started = true;
    }
    else if (_started && std::abs(latest_state.vehicle_position.lat - _start_lat) < 0.000027
            && std::abs(latest_state.vehicle_position.lon - _start_lon) < 0.000027
            && _laptime > 10.0f)
    {
        // Assume track width ~3m
        // Assume car crossed start line during race
        _lapcount++;
        _laptime = 0.0f;
    }
    else if (_started)
    {
        _laptime += time_differential;
    }
    laptime_information->set_laptime_seconds(_laptime);
    laptime_information->set_lapcount(_lapcount);

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

