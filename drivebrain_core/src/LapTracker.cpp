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

