#include <StateTracker.hpp> 
#include <hytech_msgs.pb.h> 
#include <memory>
#include <chrono>
// TODO Add any imports you might think are neccesary here

namespace core {

    struct LineSegment {
        Position start;
        Position end;

    };

    struct Point {
        double x,y;
    };
    
    class LapTracker {

        public: 

            /* Singleton move semantics */
            LapTracker(const LapTracker&) = delete;
            LapTracker& operator=(const LapTracker&) = delete;

            /**
             * Steps the lap tracker 
             * 
             * @param latest_state the latest state (will be updated in the main loop after get_latest_state_and_validity 
             * is called by the state tracker)
             */
            void step_tracker(core::VehicleState& latest_state);

            /**
             * Creates a new instance of the lap tracker
             */
            static void create(); 

            float magnitude(xyz_vec<float>& v);
            std::pair<float, float> offsetFinder(xyz_vec<float>& v);
            int orientation(Point p, Point q, Point r);
            bool onSegment(Point p, Point q, Point r);
            bool doIntersect(Point p1, Point q1, Point p2, Point q2);

            /**
             * Returns the lap tracker instance
             */
            static LapTracker& instance(); 
            void setStartLine(const Position& start, const Position& end);

        private:

            LapTracker();
        
            /** Internal State */
            inline static std::atomic<LapTracker*> _s_instance;
            
            // TODO put variables here to keep track of the lap tracker's internal state
            bool has_start_line;
            int lap_counter;
            std::chrono::steady_clock::time_point curr_lap_start_time;
            VehicleState prev_state;
            LineSegment start_line;
            bool has_crossed_start_line;
    };
    

}