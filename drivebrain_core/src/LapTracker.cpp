#include <LapTracker.hpp> 
#include <limits>
#include <cmath>
#include <cassert>
#include <chrono>
core::LapTracker::LapTracker() : has_start_line(false), lap_counter(0), has_crossed_start_line(false), has_prev_state(false), best_lap_time(std::numeric_limits<float>::max()),max_lap_speed(0.0f){
    prev_state = {};
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
    if(!has_prev_state && latest_state.state_is_valid) {
        prev_state = latest_state;
        has_prev_state=true;
        return;
    }
    else if (!has_prev_state) {
        return;
    }
    float track_width = 10.0;
    if (!has_start_line && latest_state.state_is_valid) {
        Position p0 = latest_state.vehicle_position;
        xyz_vec<float> vel = latest_state.current_body_vel_ms;
        if (magnitude(vel) > 0.1f) {
            std::pair<float, float> offset=offsetFinder(vel);
            std::pair<float, float> newOffset = {offset.first * track_width/2,offset.second*track_width/2};
            
            Position startOne = {p0.lat - meter_to_degree_latitude(newOffset.second), p0.lon - meter_to_degree_longitude(newOffset.first), true};
            Position startTwo = {p0.lat + meter_to_degree_latitude(newOffset.second), p0.lon + meter_to_degree_longitude(newOffset.first), true};
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
            if (laptime_seconds < best_lap_time) {
                best_lap_time = laptime_seconds;
            }
            laptime_information->set_best_lap_time(best_lap_time);
            laptime_information->set_delta_to_best(laptime_seconds - best_lap_time);
            laptime_information->set_laptime_seconds(laptime_seconds);
        }
        lap_counter++;
        
        curr_lap_start_time=std::chrono::steady_clock::now();
        

        has_crossed_start_line=true;
        


    }
    else if (!crossed) {
        has_crossed_start_line=false;
    }
    
    //max runtime velo
    xyz_vec<float> curr_vel = latest_state.current_body_vel_ms;
    float curr_speed = magnitude(curr_vel);
    if (curr_speed > max_lap_speed) {
        max_lap_speed = curr_speed;
    }
    laptime_information->set_max_lap_speed(max_lap_speed);
    laptime_information->set_lapcount(lap_counter);
   



    
    
    
    
    if(best_lap_time < std::numeric_limits<float>::max()) {
        laptime_information->set_best_lap_time(best_lap_time);
    }

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

float core::LapTracker::magnitude(core::xyz_vec<float> v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

//using reference latitude:  42.0676000000°N, -84.2424000000°E (michigan racetrack)
float core::LapTracker::meter_to_degree_latitude(float m) {
    constexpr float DEG_TO_RAD = static_cast<float>(M_PI)/180.0f;
    constexpr float phi = 42.06760000000f * DEG_TO_RAD;

    const float metersPerDegLat = 111132.92f - 559.82f * std::cos(2.0f * phi) + 1.175f * std::cos(4.0f * phi) - 0.0023f * std::cos(6.0f * phi);
    return m/metersPerDegLat;
}

float core::LapTracker::meter_to_degree_longitude(float m) {
constexpr float DEG_TO_RAD = static_cast<float>(M_PI) / 180.0f;
    constexpr float phi = 42.0676000000f * DEG_TO_RAD;

    const float metersPerDegLon =
        111412.84f * std::cos(phi)
        -    93.5f  * std::cos(3.0f * phi)
        +     0.118f * std::cos(5.0f * phi);

    return m / metersPerDegLon;
}


std::pair<float, float> core::LapTracker::offsetFinder(core::xyz_vec<float> v) {
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

