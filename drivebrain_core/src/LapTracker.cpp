#include <LapTracker.hpp> 
#include <cmath>
#include <cassert>
#include <chrono>
core::LapTracker::LapTracker() : has_start_line(false), lap_counter(0), has_crossed_start_line(false) {
    curr_lap_start_time = std::chrono::steady_clock::now();
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

    //Defining starting line
    //HARDCODED TRACK WIDTH VALUE (assuming car starts from middle of track)
    float track_width = 10.0;
    if (!has_start_line && latest_state.state_is_valid) {
        Position p0 = latest_state.vehicle_position;
        xyz_vec<float> vel = latest_state.current_body_vel_ms;
        if (magnitude(vel) > 0.1f) {
            std::pair<float, float> offset=offsetFinder(vel);
            std::pair<float, float> newOffset = {offset.first * track_width/2,offset.second*track_width/2};
            
            Position startOne = {p0.lat - newOffset.second, p0.lon - newOffset.first, true};
            Position startTwo = {p0.lat + newOffset.second, p0.lon + newOffset.first, true};
            setStartLine(startOne, startTwo);
            has_start_line = true;
        }
        prev_state = latest_state;
        return;
    }
    //Lap detection logic
    if(!has_start_line || !latest_state.state_is_valid) {
        return;        
    }

    Point p1 = {start_line.start.lat,start_line.start.lon};
    Point p2 = {start_line.end.lat,start_line.end.lon};
    Point p3 = {prev_state.vehicle_position.lat,prev_state.vehicle_position.lon};
    Point p4 = {latest_state.vehicle_position.lat,latest_state.vehicle_position.lon};

    
    bool crossed = doIntersect(p1,p2,p3,p4);

    if (crossed && !has_crossed_start_line) {
        auto now = std::chrono::steady_clock::now();
        if (lap_counter>0) {
            auto elapsed = std::chrono::duration<float>(now - curr_lap_start_time);
            float laptime_seconds = elapsed.count();
            laptime_information->set_laptime_seconds(laptime_seconds);
        }
        lap_counter++;
        
        curr_lap_start_time=std::chrono::steady_clock::now();


        has_crossed_start_line=true;
        laptime_information->set_lapcount(lap_counter);

    }
    else if (!crossed) {
        has_crossed_start_line=false;
    }
    


    //populate protobuf message for laptime_information

    prev_state=latest_state;
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


int core::LapTracker::orientation(core::Point p, core::Point q, core::Point r) {
    double val = (q.y-p.y) * (r.x-q.x) - (q.x-p.x) * (r.y-q.y);
    if (std::abs(val) < 0.0001) return 0;
    return (val > 0) ? 1:2;
}

bool core::LapTracker::onSegment(Point p, Point q, Point r) {
    return (q.x<=std::max(p.x,r.x) && q.x >= std::min(p.x,r.x) && q.y<=std::max(p.y,r.y) && q.y>=std::min(p.y,r.y));
}

bool core::LapTracker::doIntersect(Point p1, Point q1, Point p2, Point q2) {
    int o1 = orientation(p1,q1,p2);
    int o2 = orientation(p1,q1,q2);
    int o3 = orientation(p2,q2,p1);
    int o4 = orientation(p2,q2,q1);

    if(o1 != o2 && o3 != o4) return true;

    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
    return false;
}

float core::LapTracker::magnitude(core::xyz_vec<float>& v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}



std::pair<float, float> core::LapTracker::offsetFinder(core::xyz_vec<float>& v) {
    float mag = magnitude(v);
    return {(-v.y/mag),(v.x/mag)};
}


//setter to set start line
void core::LapTracker::setStartLine(const Position& start, const Position& end) {
    start_line.start=start;
    start_line.end=end;
}


core::LapTracker& core::LapTracker::instance() {
    LapTracker* instance = _s_instance.load(std::memory_order_acquire);
    assert(instance != nullptr && "LapTracker has not been initialized");
    return *instance;
}

