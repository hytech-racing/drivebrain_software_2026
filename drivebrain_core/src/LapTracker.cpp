#include <LapTracker.hpp> 
#include <cmath>

void core::LapTracker::step_tracker(core::VehicleState& latest_state) {
    std::shared_ptr<hytech_msgs::LapTime> laptime_information = std::make_shared<hytech_msgs::LapTime>(); // TODO you need to fill in the fields of this protobuf message

    /**
     * TODO it is your responsibility to fill in this method. Look at the VehicleState 
     * struct to see all the things it gives you (you should see vn position from there). Using this
     * and the private variables you added in the header, complete this method. It should use all of the information
     * in the latest state to update it's local variables, create a LapTime protobuf, and invoke handle_receive_protobuf_message
     * on the state tracker. Some of this is completed for you. Good luck!
     */

     auto& currPos = latest_state.vehicle_position;

     // error checking
     if (!latest_state.state_is_valid) {
        return;
     }

     auto clock = std::chrono::steady_clock::now();
     float current_yaw = latest_state.current_ypr_rad.yaw;

     // start reading and mark start and exit method
     if (!start_set) {
        start_position = currPos;
        current_position = currPos;
        start_set = true;
        start_time = clock;
        start_yaw = latest_state.current_ypr_rad.yaw;
        return;
     }

     // distance calculation
     double distance = distance_formula(start_position, currPos);

     // set valid lap, consider orientation using yaw
     if (distance == 0 && current_yaw == start_yaw) {
        valid_lap = true;
     }

     // valid lap, 
     if (valid_lap) {
        auto end_time = clock;

        // calculate lap time
        std::chrono::duration<double> diff = end_time - start_time;
        double elapsed = diff.count();

        // send lap time
        laptime_information -> set_lapcount(++num_laps);
        laptime_information -> set_laptime_seconds(elapsed);
        core::StateTracker::instance().handle_receive_protobuf_message(laptime_information); // What "records" the information

        //reset info
        start_time = clock;
        valid_lap = false;
     }

    // vehicle position, includes latitude, longitude, and valid
    // keep track of time at each new state
    // keep track of orientation of car to see if its a valid lap, aka no turning around and coming back
    // populate laptime info

    // how to access from latest state, make private variables

    // lap_start_time = std::chrono::steady_clock::now();


    // // start - end time to get elapsed for the lap
    // // when to measure end time if the function is called on the start of each lap?
    // // when the lon or lat is equal to start? but idk if thats accurate

    // laptime_information -> set_lapcount(num_lap++);
    // laptime_information -> set_laptime_seconds(); // reset after each lap is finished

}

float core::LapTracker::distance_formula(core::Position& start, core::Position& curr) {
    if (!start.valid || !curr.valid) {
        return;
    }

    float R = 6371000.0;

    //Haversine formula or distance formula
    double lat = (curr.lat - start.lat) * (M_PI/180);
    double lon = (curr.lon - start.lon) * (M_PI/180);
    double latSRad = start.lat * (M_PI/180);
    double latCRad = curr.lat * (M_PI/180);


    double a = sin(lat/2) * sin(lat/2) + cos(latSRad) * cos(latCRad) *  sin(lon/2) * sin(lon/2);
    double b = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * b; //distance in m after multiplying by radius of earth in m
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

